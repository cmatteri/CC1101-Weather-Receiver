# Copyright 2014 Chris Matteri
# Released under the MIT License (http://opensource.org/licenses/mit-license.php)

import time

import weewx
import weeutil.weeutil

import ccwxrxbase

def loader(config_dict, engine):
    return CCWXRXVP2(**config_dict['CCWXRXVP2'])

class CCWXRXVP2(ccwxrxbase.CCWXRXBase):
    """weewx driver for the Davis Vantage Pro 2 used with the CC1101 Weather
    Receiver."""

    def __init__(self, **stn_dict):
        super(CCWXRXVP2, self).__init__(**stn_dict)

        self.gust_packet_period = 20 * self.poll_interval
        self.last_gust_time = None
        self.last_rain = None

        # This driver receives most of its data the VP2 ISS, but pressure
        # data comes directly from the CCWXRX.  A ConnectionThread is required
        # for each data source.
        self.iss_data = ccwxrxbase.ConnectionThread(kwargs={
            'hostname': self.ccwxrx_splitter_hostname,
            'port': self.ccwxrx_splitter_port,
            'transmitter_id': self.transmitter_id})
        self.iss_data.daemon = True
        self.iss_data.start()
        # Pressure data is measured by the CCWXRX and is available on the
        # pseudo transmitter_id 0.
        self.pressure_data = ccwxrxbase.ConnectionThread(kwargs={
            'hostname': self.ccwxrx_splitter_hostname,
            'port': self.ccwxrx_splitter_port,
            'transmitter_id': 0})
        self.pressure_data.daemon = True
        self.pressure_data.start()

    def genLoopPackets(self):
        while True:
            packet = {'dateTime': int(time.time() + 0.5),
                       'usUnits': weewx.US,
                       }
            with self.iss_data.lock:
                mesg = self.iss_data.mesg
                self.iss_data.mesg = None

            if mesg is not None:
                if weewx.debug:
                    ccwxrxbase.logdbg('ISS message received: ' + mesg)
                try:
                    data_packet = self.read_packet(mesg)
                    ccwxrxbase.logdbg('CRC succeeded')

                    wind_dir = data_packet[2] * 360 / 255
                    if wind_dir == 0:
                        ccwxrxbase.logerr('Wind direction was not reported. '
                               'Wind vane may require maintenance.')
                    else:
                        packet['windDir'] = wind_dir
                    packet['windSpeed'] = data_packet[1]
                    data_type = (data_packet[0] & 0xf0) >> 4
                    # Temperature
                    if data_type == 6:
                        packet['radiation'] = self.calculate_solar_radiation(
                            data_packet)
                    elif data_type == 8:
                        packet['outTemp'] = self.calculate_temp(data_packet)
                    elif data_type == 9:
                        wind_gust = self.calculate_wind_gust(data_packet)
                        if wind_gust is not None:
                            # Wind gust data from message 9 has less precise
                            # time information than wind speed data from byte
                            # 1, and the wind direction used is potentially out
                            # of date.  Unless the gust is stronger than a wind
                            # speed reading from the current archive interval,
                            # it will be ignored by weewx, which is the desired
                            # behavior (the exception being if the only wind
                            # speed greater than or equal to the wind gust is
                            # from the same packet that reported that wind
                            # gust, but that's not a problem because the timing
                            # and direction data will be the same from either
                            # source in such a case. See accum.py for details).
                            # In other words, wind gust data is only used if
                            # the packet reporting the wind speed corresponding
                            # to that gust was missed. See
                            # https://github.com/dekay/DavisRFM69/wiki/Message-Protocol
                            # for more details.
                            packet['windGust'] = wind_gust
                            # This is an estimate for the direction of the wind
                            # gust, since the ISS does not appear to transmit
                            # the exact direction.
                            packet['windGustDir'] = wind_dir
                    elif data_type == 0xa:
                        packet['outHumidity'] = self.calculate_humidity(
                            data_packet)
                    elif data_type == 0xe:
                        packet['rain'] = self.calculate_rain(data_packet)
                except ccwxrxbase.CommunicationError as e:
                    ccwxrxbase.logerr(e)

            with self.pressure_data.lock:
                mesg = self.pressure_data.mesg
                self.pressure_data.mesg = None

            if mesg is not None:
                try:
                    data_packet = self.read_packet(mesg)
                    packet['pressure'] = self.calculate_pressure(data_packet)
                except ccwxrxbase.CommunicationError as e:
                    ccwxrxbase.logerr(e)
            if weewx.debug:
                ccwxrxbase.logdbg('Loop packet: ' + str(packet))
            yield packet
            time.sleep(self.poll_interval)

    @property
    def hardware_name(self):
        return "CCWXRXVP2"

    def calculate_solar_radiation(self, data_packet):
        return (((data_packet[3] << 8) + data_packet[4]) >> 6) * 1.757936

    def calculate_temp(self, data_packet):
        return ((data_packet[3] << 8) + data_packet[4]) / 160.0

    def calculate_wind_gust(self, data_packet):
        gust_time = time.time()
        gust = None
        if self.last_gust_time is not None:
            elapsed_packets = round((gust_time - self.last_gust_time) /
                              self.gust_packet_period)
            gust_counter = data_packet[5] >> 4
            if gust_counter < elapsed_packets:
                gust = data_packet[3]
        self.last_gust_time = gust_time
        return gust

    def calculate_humidity(self, data_packet):
        return (((data_packet[4] & 0xf0) << 4) + data_packet[3]) / 10.0

    def calculate_rain(self, data_packet):
        rain = data_packet[3]
        if self.last_rain is not None:
            if rain == self.last_rain:
                inches_rain = 0
            else:
                inches_rain = (rain - self.last_rain) % 128 * 0.01
        else:
            inches_rain = 0
        self.last_rain = rain
        return inches_rain

    def calculate_pressure(self, data_packet):
        pressure_pa = ((data_packet[1] << 24) + (data_packet[2] << 16) +
                       (data_packet[3] << 8) + (data_packet[4]))
        return pressure_pa * 0.000295333727  # Convert to inHg
