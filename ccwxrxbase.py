# Copyright 2014 Chris Matteri
# Released under the MIT License (http://opensource.org/licenses/mit-license.php)

import multiprocessing.connection
import sys
import syslog
import threading
import time

import weewx.crc16
import weewx.drivers
import weeutil.weeutil

class CommunicationError(Exception):
    pass

def logmsg(dst, msg):
    syslog.syslog(dst, 'ccwxrxvp2: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

class ConnectionThread(threading.Thread):
    """Thread that receives data packets for a specific transmitter ID from
    the ccwxrx_splitter program."""

    def __init__(self, group=None, name=None, kwargs=None):
        super(ConnectionThread, self).__init__(group=group, name=name)
        self.hostname = kwargs['hostname']
        self.port = kwargs['port']
        self.transmitter_id = kwargs['transmitter_id']
        self.lock = threading.Lock()
        self.mesg = None
        return

    def run(self):
        conn = multiprocessing.connection.Client((self.hostname, self.port))
        conn.send(self.transmitter_id)
        while True:
            mesg = conn.recv()
            with self.lock:
                self.mesg = mesg
        return

class CCWXRXBase(weewx.drivers.AbstractDevice):

    def __init__(self, **stn_dict):
        # How often to call genLoopPackets.
        self.poll_interval = float(stn_dict.get('poll_interval', 2.5))
        self.ccwxrx_splitter_hostname = stn_dict.get(
            'ccwxrx_splitter_hostname', '127.0.0.1')
        self.ccwxrx_splitter_port = int(stn_dict.get('ccwxrx_splitter_port',
                                                    5772))
        self.transmitter_id = int(stn_dict.get('transmitter_id', 1))

        self.transmission_period = 2.5 + (self.transmitter_id - 1)*0.5/7

    def read_packet(self, mesg):
        try:
            # bytearray is used here to cause a ValueError
            # exception if any element in the generated list is
            # greater than 255.
            data_packet = bytearray(
                [int(i, base=16) for i in mesg.split()])
        except ValueError:
            raise CommunicationError(
                'Invalid message received: {}'.format(mesg))
        if len(data_packet) != 8:
            raise CommunicationError(
                'Invalid message received: {}'.format(mesg))
        crc = weewx.crc16.crc16(
            ''.join(map(chr, data_packet[0:6])))
        if crc != (data_packet[6] << 8) + data_packet[7]:
            raise CommunicationError('CRC failed.')
        return data_packet
