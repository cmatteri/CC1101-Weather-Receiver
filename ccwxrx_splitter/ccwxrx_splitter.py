#!/usr/bin/env python2
# Copyright 2014 Chris Matteri
# Released under the MIT License (http://opensource.org/licenses/mit-license.php)

import atexit
import logging
import multiprocessing.connection
import os
import Queue
import serial
from signal import SIGTERM
import socket
import sys
import threading
import time
import traceback

serial_device = '/dev/ttyAMA0'
serial_baud = '19200'
hostname = '127.0.0.1'
port = 5772
log_level = logging.WARNING
# Set to None to print logging output to stdout
log_file = '/var/log/ccwxrx_splitter.log'
pid_file = '/var/run/ccwxrx_splitter.pid'

# Modified from http://www.jejik.com/articles/2007/02/a_simple_unix_linux_daemon_in_python/
def daemonize(pidfile, stdin='/dev/null', stdout='/dev/null',
 stderr='/dev/null'):
    """
    do the UNIX double-fork magic, see Stevens' "Advanced
    Programming in the UNIX Environment" for details (ISBN 0201563177)
    http://www.erlenstar.demon.co.uk/unix/faq_2.html#SEC16
    """
    try:
        pid = os.fork()
        if pid > 0:
                # exit first parent
                sys.exit(0)
    except OSError, e:
        sys.stderr.write("fork #1 failed: %d (%s)\n" % (e.errno, e.strerror))
        sys.exit(1)

    # decouple from parent environment
    os.chdir("/")
    os.setsid()
    os.umask(0)

    # do second fork
    try:
        pid = os.fork()
        if pid > 0:
                # exit from second parent
                sys.exit(0)
    except OSError, e:
        sys.stderr.write("fork #2 failed: %d (%s)\n" % (e.errno, e.strerror))
        sys.exit(1)

    # redirect standard file descriptors
    sys.stdout.flush()
    sys.stderr.flush()
    si = file(stdin, 'r')
    so = file(stdout, 'a+')
    se = file(stderr, 'a+', 0)
    os.dup2(si.fileno(), sys.stdin.fileno())
    os.dup2(so.fileno(), sys.stdout.fileno())
    os.dup2(se.fileno(), sys.stderr.fileno())

    # write pidfile
    pid = str(os.getpid())
    file(pidfile,'w+').write("%s\n" % pid)

class CommunicationError(Exception):
    pass


class ProgramError(Exception):
    pass

class ConnectionThread(threading.Thread):
    def __init__(self, group=None, name=None, kwargs=None):
        super(ConnectionThread, self).__init__(group=group, name=name)
        self.conn = kwargs['conn']
        self.ready = threading.Event()
        self.queue = Queue.Queue()
        self.id = None
        return

    def run(self):
        logging.info('Client {} connected.'.format(self.name))
        try:
            self.id = int(self.conn.recv())
        except ValueError:
            logging.error('Invalid message received from client {}. Closing '
                          'connection.'.format(self.name))
            self.conn.close()
            return
        self.ready.set()
        logging.info('Client {} subscribed to transmitter '
                     'id {}.'.format(self.name, self.id))
        try:
            while True:
                msg = self.queue.get()
                self.conn.send(msg)
        except IOError:
            self.conn.close()
            logging.info('Client {} connection broken.'.format(self.name))
        return

class ServerThread(threading.Thread):
    def __init__(self, kwargs):
        super(ServerThread, self).__init__()
        self.hostname = kwargs['hostname']
        self.port = kwargs['port']
        self.connection_threads = kwargs['connection_threads']
        return

    def run(self):
        listener = multiprocessing.connection.Listener((self.hostname,
                                                        self.port))
        client_count = 0
        while True:
            conn = listener.accept()
            client_count += 1
            ct = ConnectionThread(kwargs={'conn': conn}, name=client_count)
            ct.daemon = True
            ct.start()
            self.connection_threads.append(ct)

def log_traceback(trace):
    """Log a python stack trace to syslog.
       From: http://stackoverflow.com/questions/18348183"""

    log_lines = trace.split('\n')
    for line in log_lines:
        if len(line):
            logging.critical(line)

def check_alive(ser):
    ser.write('strmoff\n')  # Turn data stream off
    time.sleep(0.1)  # Wait for strmoff command to take effect
    ser.flushInput()  # Discard any input sent before strmoff took effect
    ser.write('test\n')
    ser.timout = 0.1
    line = ser.readline

def setup(ser):
    tries = 0
    while True:
        tries += 1
        ser.write('strmoff\n')  # Turn data stream off
        time.sleep(0.1)  # Wait for strmoff command to take effect
        ser.flushInput()  # Discard any input sent before strmoff took effect
        ser.write('strmon\n')
        ser.timeout = 0.1  # 0.1 second receive timeout
        line = ser.readline()
        if line == 'data stream on\r\n':
            break
        elif tries == 3:
            raise CommunicationError('ccwxrx is not responding.')
        else:
            logging.warning('ccwxrx did not respond to strmon command.')
    logging.info('Data link to ccwxrx established.')
    ser.timeout = 90

def main():
    daemonize(pid_file)
    # Build a dictionary of keyword args for logging.basicConfig
    kwargs = \
        {'level': log_level,
         'format': '%(asctime)s %(levelname)s: %(message)s'}
    if log_file is not None:
        kwargs['filename'] = log_file

    logging.basicConfig(**kwargs)

    connection_threads = []

    kwargs={'hostname': hostname, 'port': port,
            'connection_threads': connection_threads}
    server_thread = ServerThread(kwargs)
    server_thread.daemon = True
    server_thread.start()

    ser = serial.Serial(serial_device, serial_baud)

    setup(ser)

    while True:
        line = ser.readline()
        logging.debug('mesg from serial: {}'.format(repr(line)))
        if line == '':
            logging.warning('No data received after 90 seconds. '
                            'Reestablishing data link')
            setup(ser)

        try:
            # Strip trailing return and newline chars.
            line = line[:-2]

            if line.find('error:') != -1:
                logging.error(line[7:])
                continue
            
            byte0 = int(line[0:line.index(' ')], base=16)
            # The value in the 3 lowest order bits of the first byte of the
            # data packet contains the transmitter ID minus one, unless byte 0
            # is 0, in which case the packet contains pressure data, which is
            # broadcast by this program on the 0 pseudo-ID.
            if byte0 == 0:
                transmitter_id = 0
            else:
                transmitter_id = (byte0 & 7) + 1
        except ValueError:
            logging.warning('ccwxrx sent invalid message')
            continue
        for thread in connection_threads:
            if not thread.isAlive():
                connection_threads.remove(thread)
            elif thread.ready.isSet() and thread.id == transmitter_id:
                thread.queue.put(line)

if __name__ == '__main__':
    try:
        main()
    except SystemExit:
        pass
    except:
        log_traceback(traceback.format_exc())
