// Copyright (c) 2014 Chris Matteri
// Released under the MIT License (http://opensource.org/licenses/mit-license.php)

#include "weather_device.h"

namespace cc1101_weather_receiver {

WeatherDevice::WeatherDevice()
  : is_active(false),
    is_synchronized(false),
    next_packet_channel(false),
    packets_received(0),
    packets_missed(0),
    num_resyncs(0),
    consecutive_missed(0) {}

} // namespace cc1101_weather_receiver
