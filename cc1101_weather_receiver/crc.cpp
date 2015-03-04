#include "crc.h"

namespace cc1101_weather_receiver {

// From http://www.menie.org/georges/embedded/
uint16_t CRC16_CCITT(const uint8_t *buf, int len) {
  uint16_t crc = 0;
  while (len--) {
    uint16_t i;
    crc ^= *buf++ << 8;
    for (i = 0; i < 8; ++i) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

} // namespace cc1101_weather_receiver
