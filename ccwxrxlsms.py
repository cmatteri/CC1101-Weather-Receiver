# Copyright 2014 Chris Matteri
# Released under the MIT License (http://opensource.org/licenses/mit-license.php)

import math
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

def loader(config_dict, engine):
    return CCWXRXLSMS(**config_dict['CCWXRXLSMS'])

def logmsg(dst, msg):
    syslog.syslog(dst, 'ccwxrxlsms: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

class ConnectionThread(threading.Thread):
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

class CCWXRXLSMS(weewx.drivers.AbstractDevice):
    """weewx driver for the Davis Vantage Pro 2 used with the CC1101 Weather
    Receiver."""

    def __init__(self, **stn_dict):
        # how often to poll the weather data file, seconds
        self.poll_interval = float(stn_dict.get('poll_interval', 2.5))
        self.ccwxrx_splitter_hostname = stn_dict.get(
            'ccwxrx_splitter_hostname', '127.0.0.1')
        self.ccwxrx_splitter_port = int(stn_dict.get('ccwxrx_splitter_port',
                                                    5772))
        self.transmitter_id = int(stn_dict.get('transmitter_id', 2))
        # To enable soil moisture sensors 1 3 and 4, put soil_moist='1 3 4' in
        # the weewx conf file.
        self.soil_moist_enabled = map(int, stn_dict.get(
            'soil_moist', '').split())
        self.soil_temp_enabled = map(int, stn_dict.get(
            'soil_temp', '').split())
        # Default soil temp in degrees celsius.
        self.default_soil_temp = stn_dict.get('default_soil_temp', 24)

        self.lsm_data = ConnectionThread(kwargs={
            'hostname': self.ccwxrx_splitter_hostname,
            'port': self.ccwxrx_splitter_port,
            'transmitter_id': self.transmitter_id})
        self.lsm_data.daemon = True
        self.lsm_data.start()

    def genLoopPackets(self):
        while True:
            # map the data into a weewx loop packet
            packet = {'dateTime': int(time.time() + 0.5),
                       'usUnits': weewx.METRIC,
                       }
            with self.lsm_data.lock:
                mesg = self.lsm_data.mesg
                self.lsm_data.mesg = None

            if mesg is not None:
                if weewx.debug:
                    logdbg('Leaf and soil moisture message received:')
                    logdbg(mesg)
                try:
                    data_packet = self.read_packet(mesg)
                    logdbg('CRC succeeded')

                    data_type = data_packet[0] >> 4
                    if data_type != 0xf:
                        raise CommunicationError(
                            'Expected message 0xf, received message {}  Is '
                            'the transmitter ID set correctly?'.format(
                            data_type))
                    sensor_num = ((data_packet[1] & 0xe0) >> 5) + 1
                    if sensor_num < 1 or sensor_num > 4:
                        raise CommunicationError(
                            'sensor_num (value is {}) number is out of range'
                            ''.format(sensor_num))
                    data_subtype = data_packet[1] & 0x3
                    if data_subtype == 1:
                        soil_temp_raw = ((data_packet[3] << 2) +
                                         (data_packet[5] >> 6))
                        soil_potential_raw = ((data_packet[2] << 2) +
                                              (data_packet[4] >> 6))
                        # soil_temp_raw and soil_potential_raw are set to their
                        # max values (0x3ff) when the sensor is not populated.
                        soil_temp = None
                        if sensor_num in self.soil_temp_enabled:
                            soil_temp = self.calculate_soil_temp(soil_temp_raw)
                            packet['soilTemp{}'.format(sensor_num)] = soil_temp

                        if sensor_num in self.soil_moist_enabled:
                            if soil_temp is None:
                                soil_temp = self.default_soil_temp
                            packet['soilMoist{}'.format(sensor_num)] = \
                                self.calculate_soil_potential(
                                    soil_potential_raw, soil_temp)
                except CommunicationError as e:
                    logerr(e)

            if weewx.debug:
                logdbg(str(packet))
            yield packet
            time.sleep(self.poll_interval)

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

    def calculate_soil_temp(self, soil_temp_raw):
        logdbg('soil_temp_raw: {}'.format(soil_temp_raw))
        # temp is in degrees C
        # R is in kohms

        # See soil_moisture_protocol.txt for details on how resistance is
        # calculated
        A = 18.81099
        B = 0.0009988027
        R = A / (1.0/soil_temp_raw - B) / 1000
        if weewx.debug:
            logdbg('thermistor resistance: {} kohm'.format(R))

        # Steinhart-Hart parameters.
        S1 = 0.002783573
        S2 = 0.0002509406
        try:
            return 1/(S1 + S2*math.log(R)) - 273
        except ValueError:
            logerr("soil_temp_raw: {}".format(soil_temp_raw))
            return 24

    def calculate_soil_potential(self, soil_potential_raw, soil_temp):
        # Equations relating resistance to soil potential are from
        # http://www.kimberly.uidaho.edu/water/swm/Calibration_Watermark2.htm
        # The following units are used to make it easier to verify that the
        # equations here match those from the above link
        # soil_temp is in degrees C
        # R is in kohms
        # potential is in kPa (equivalently to centibar)

        # See soil_moisture_protocol.txt for details on how resistance is
        # calculated.
        A = 13.50903
        B = 0.001070697
        R = A / (1.0/soil_potential_raw - B) / 1000
        if weewx.debug:
            logdbg('soil moisture sensor resistance: {} kohm'.format(R))

        if R <= 0:
            raise CommunicationError('Calculated resistance of soil moisture '
                                     'sensor is out of range.')
        if R < 1:
            potential = -20 * (R * (1 + 0.018*(soil_temp - 24)) - 0.55)
        elif R < 8:
            potential = (-3.213*R - 4.093) / \
                       (1 - 0.009733*R - 0.01205*soil_temp)
        else:
            potential = -2.246 - 5.239*R*(1 + 0.018*(soil_temp - 24)) \
                       - 0.06756*R**2*(1 + 0.018*(soil_temp - 24))**2
        return potential