// Copyright (c) 2014-2015 Chris Matteri
// Released under the MIT License (http://opensource.org/licenses/mit-license.php)

#include "weather_device.h"

namespace cc1101_weather_receiver {

WeatherDevice::WeatherDevice()
  : is_active(false),
    is_tracked(false),
    next_packet_channel(false),
    consecutive_missed(0) {}

} // namespace cc1101_weather_receiver
