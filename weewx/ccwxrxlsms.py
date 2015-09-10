# Copyright 2014 Chris Matteri
# Released under the MIT License (http://opensource.org/licenses/mit-license.php)

import math
import time

import weewx
import weeutil.weeutil

import ccwxrxbase

def loader(config_dict, engine):
    return CCWXRXLSMS(**config_dict['CCWXRXLSMS'])

class CCWXRXLSMS(ccwxrxbase.CCWXRXBase):
    """weewx driver for the Davis Vantage Pro 2 used with the CC1101 Weather
    Receiver."""

    def __init__(self, **stn_dict):
        super(CCWXRXLSMS, self).__init__(**stn_dict)

        # To enable soil moisture sensors 1 3 and 4, put soil_moist='1 3 4' in
        # the weewx conf file.
        self.soil_moist_enabled = map(int, stn_dict.get(
            'soil_moist', '').split())
        self.soil_temp_enabled = map(int, stn_dict.get(
            'soil_temp', '').split())
        # Default soil temp in degrees celsius.
        self.default_soil_temp = stn_dict.get('default_soil_temp', 24)

        self.lsm_data = ccwxrxbase.ConnectionThread(kwargs={
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
                    ccwxrxbase.logdbg(
                            'Leaf and soil moisture message received:')
                    ccwxrxbase.logdbg(mesg)
                try:
                    data_packet = self.read_packet(mesg)
                    ccwxrxbase.logdbg('CRC succeeded')

                    data_type = data_packet[0] >> 4
                    if data_type != 0xf:
                        raise ccwxrxbase.CommunicationError(
                            'Expected message 0xf, received message {}  Is '
                            'the transmitter ID set correctly?'.format(
                            data_type))
                    sensor_num = ((data_packet[1] & 0xe0) >> 5) + 1
                    if sensor_num < 1 or sensor_num > 4:
                        raise ccwxrxbase.CommunicationError(
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
                except ccwxrxbase.CommunicationError as e:
                    ccwxrxbase.logerr(e)

            if weewx.debug:
                ccwxrxbase.logdbg(str(packet))
            yield packet
            time.sleep(self.poll_interval)

    def calculate_soil_temp(self, soil_temp_raw):
        ccwxrxbase.logdbg('soil_temp_raw: {}'.format(soil_temp_raw))
        # temp is in degrees C
        # R is in kohms

        # See soil_moisture_protocol.txt for details on how resistance is
        # calculated
        A = 18.81099
        B = 0.0009988027
        R = A / (1.0/soil_temp_raw - B) / 1000
        if weewx.debug:
            ccwxrxbase.logdbg('thermistor resistance: {} kohm'.format(R))

        # Steinhart-Hart parameters.
        S1 = 0.002783573
        S2 = 0.0002509406
        try:
            return 1/(S1 + S2*math.log(R)) - 273
        except ValueError:
            ccwxrxbase.logerr("soil_temp_raw: {}".format(soil_temp_raw))
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
            ccwxrxbase.logdbg(
                    'soil moisture sensor resistance: {} kohm'.format(R))

        if R <= 0:
            raise ccwxrxbase.CommunicationError(
                    'Calculated resistance of soil moisture sensor is out of '
                    'range.')
        if R < 1:
            potential = -20 * (R * (1 + 0.018*(soil_temp - 24)) - 0.55)
        elif R < 8:
            potential = (-3.213*R - 4.093) / \
                       (1 - 0.009733*R - 0.01205*soil_temp)
        else:
            potential = -2.246 - 5.239*R*(1 + 0.018*(soil_temp - 24)) \
                       - 0.06756*R**2*(1 + 0.018*(soil_temp - 24))**2
        return potential
