# Copyright 2014 Chris Matteri
# Released under the MIT License (http://opensource.org/licenses/mit-license.php)

import logging
import multiprocessing.connection
import Queue
import serial
import socket
import sys
import threading
import time

serial_device = '/dev/ttyAMA0'
serial_baud = '19200'
hostname = '127.0.0.1'
port = 5772
log_level = logging.DEBUG
# Set to None to print logging output to stdout
log_file = '/home/chris/splitter_log'

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

def get_lock():
    """From: http://stackoverflow.com/a/7758075/1197903"""
    global lock_socket  # Make global to prevent garbage collection and closing
                        # when get_lock returns
    lock_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        lock_socket.bind('\0ccwxrx_splitter')
        logging.debug('Got the lock.')
    except socket.error:
        raise ProgramError('Could not get lock. ccwxrx_splitter is probably '
                           'already running.')
        sys.exit()

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
    # Build a dictionary of keyword args for logging.basicConfig
    kwargs = \
        {'level': log_level,
         'format': '%(asctime)s %(levelname)s: %(message)s'}
    if log_file is not None:
        kwargs['filename'] = log_file

    logging.basicConfig(**kwargs)

    get_lock()

    connection_threads = []

    server_thread = ServerThread(kwargs={'hostname': hostname, 'port': port,
                                         'connection_threads': connection_threads})
    server_thread.daemon = True
    server_thread.start()

    ser = serial.Serial(serial_device, serial_baud)

    setup(ser)

    while True:
        line = ser.readline()
        logging.debug('mesg from serial: {}'.format(repr(line)))
        if line == '':
            logging.warning('No data received after 90 seconds. Reestablishing '
                            'data link')
            setup(set)

        try:
            # Strip trailing return and newline chars.
            line = line[:-2]

            if line.find('error:') != -1:
                logging.error(line)
                continue
            
            space_i = line.index(' ')
            byte0 = int(line[0:space_i], base=16)
            # The value in the 3 lowest order bits of the first byte of the data
            # packet contains the transmitter ID minus one.
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

main()
