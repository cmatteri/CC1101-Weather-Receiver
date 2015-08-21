// Copyright (c) 2014-2015 Chris Matteri
// Released under the MIT License (http://opensource.org/licenses/mit-license.php)

#ifndef WEATHER_DEVICE_H_
#define WEATHER_DEVICE_H_

#include <Arduino.h>
#include <stdint.h>

namespace cc1101_weather_receiver {

struct WeatherDevice {
  WeatherDevice();
  byte id;  // This device's transmitter ID.
  bool is_active;  // Determines whether the receiver listens for a transmitter
                   // with this ID.
  bool is_tracked;  // Indicates whether the receiver is tracking this
                    // device's transmissions, i.e. whether the next packet's
                    // channel and time of arrival are known.
  byte next_packet_channel;
  uint32_t transmission_period;  // Delay between transmissions in microseconds
  uint32_t next_channel_switch_time;
  uint16_t consecutive_missed;
};

} // namespace cc1101_weather_receiver

#endif // WEATHER_DEVICE_H_
