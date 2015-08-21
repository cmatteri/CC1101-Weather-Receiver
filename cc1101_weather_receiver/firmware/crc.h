#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

namespace cc1101_weather_receiver {

uint16_t CRC16_CCITT(const uint8_t *buf, int len);

} // namespace cc1101_weather_receiver

#endif // CRC_H_
