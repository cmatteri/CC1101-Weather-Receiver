// Copyright (c) 2014 Chris Matteri
// Released under the MIT License (http://opensource.org/licenses/mit-license.php)

#include <Adafruit_BMP085.h>  // Works with BMP180 as well.
#include <Arduino.h>
#include <SerialCommand.h>  // from: https://github.com/dekay/Arduino-SerialCommand
#include <SPI.h>  // Needed here for Arduino to add this header to the include
                  // path.  Used in cc1101_module.h.
#include <TimerOne.h>  // Needed here for Arduino to add this header to the
                       // include path.  Used in cc1101_module.h.
#include <Wire.h>

#include "cc1101_weather_receiver.h"
#include "crc.h"

const uint32_t kSerialBaud = 19200;
const uint16_t kPressurePeriod = 60000;  // Pressure data transmission rate in
                                          // milliseconds.

bool stream_on = false;  // Determines whether to send weather data over the
                            // serial interface.
bool debug_on = false;  // Determines whether to print debugging information on
                        // the serial interface.
int32_t last_pressure_cycle = -1;  // Used to determine when to transmit
                                   // pressure data.  See code in loop().

using cc1101_weather_receiver::CC1101Module;

CC1101Module radio;
Adafruit_BMP085 bmp;
SerialCommand scmd;  // Dekay's modified version of the SerialCommand library.

void setup() {
  Serial.begin(kSerialBaud);

  // Set transmitter IDs to track.  A value of 1 sets the ID to active.
  // These IDs correspond to the IDs set on the weather instruments.
  const bool activeIds[CC1101Module::kNumIds] =
  //    ID:
  //    1  2  3  4  5  6  7  8
      { 1, 1, 1, 0, 0, 0, 0, 0 };
  // Non-US frequencies are not implemented.  I believe EU frequencies are known.
  // Email me if you would like me to implement them,  I have a spreadsheet ready
  // to calculate them, but I would need help testing them.
  radio.Initialize(activeIds, CC1101Module::kUS);

  // Set up BMP180 pressure and temperature sensor.
  if (!bmp.begin()) {
    Serial.println(F("error: initializing BMP180 failed."));
    while (true);
  }

  // Setup callbacks for SerialCommand commands.
  // Only the first 8 characters of each command will be considered.
  // The command and arguments must together be less than 32 characters.
  scmd.addCommand("strmon", CmdStreamOn);  // Enable transmission of weather
                                           // data on the serial interface.
  scmd.addCommand("strmoff", CmdStreamOff);  // Disable transmission of weather
                                             // data on the serial interface.
  scmd.addCommand("test", CmdTest);  // Echo "test".  Beware that if your
                                     // serial client does not have local echo
                                     // turned on this command may appear to do
                                     // nothing to the untrained eye.
  // Debugging commands
  scmd.addCommand("dbgon", CmdDebugOn);  // Enable printing of debugging
                                         // information.
                                         // By default, the serial buffer is
                                         // only 64 bytes.  Once it is full
                                         // writing to serial will block until
                                         // there is room in the buffer.
                                         // Debug printing must use a minimal
                                         // character count to reduce the
                                         // likelihood of filling the serial
                                         // buffer when multiple data packets
                                         // are received in a short time
                                         // period.
                                         // Use caution when adding additional
                                         // debugging statements.
  scmd.addCommand("dbgoff", CmdDebugOff);  // Disable printing of debugging
                                           // information.
  scmd.addCommand("rreg", CmdReadReg);  // Usage: rreg <addr>
                                        // Prints the value of the CC1101
                                        // register at the address specified by
                                        // the decimal value <addr>.
                                        // The value is printed in hex.
  scmd.addCommand("wreg", CmdWriteReg);  // Usage: wreg <addr> <val>
                                         // Write the decimal value <val> to the
                                         // CC1101 register at the address
                                         // specified by the decimal value
                                         // <addr>.
  scmd.addCommand("strobe", CmdStrobe);  // Usage: strobe <addr>
                                         // Send strobe specified by the decimal
                                         // value <addr>.
  scmd.addCommand("freeram", CmdFreeRam);  // Print free RAM.

  scmd.setDefaultHandler(CmdUnrecognized);  // Handler for command that isn't
                                            // matched.  Print error message.
  scmd.setNullHandler(CmdNull);  // Do nothing on empty message.

  // Activate internal pullups on unused pins.
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A6, HIGH);
  digitalWrite(A7, HIGH);
}


void loop() {
  scmd.readSerial(); // Process serial commands.
  radio.Update();

  if (radio.PacketReady() && stream_on) {
    SendPacket(radio.packet());
  }

  int32_t current_pressure_cycle = millis() / kPressurePeriod;
  if(current_pressure_cycle != last_pressure_cycle) {
    if (stream_on) {
      // Transmit pressure in an 8 byte packet.  The first byte being 0
      // indicates that the packet contains pressure data.  Bytes 1 to 4 contain
      // the pressure data, byte 5 is unused, and bytes 6 and 7 contain the CRC
      // of bytes 0 to 5.
      int32_t pressure = bmp.readPressure();  // In Pa.
      byte pressure_packet[8] = {0,
          pressure >> 32,
          pressure >> 16,
          pressure >> 8,
          pressure};
      uint16_t crc = cc1101_weather_receiver::CRC16_CCITT(pressure_packet, 6);
      pressure_packet[6] = crc >> 8;
      pressure_packet[7] = crc;
      SendPacket(pressure_packet);
    }
    last_pressure_cycle = current_pressure_cycle;
  }
}


void SendPacket(const byte *packet) {
  uint8_t i;
  for (i = 0; i < CC1101Module::kPacketSize - 1; i++) {
    Serial.print(packet[i], HEX);
    Serial.write(' ');
  }
  Serial.println(packet[i], HEX);
}


void CmdStreamOn() {
  stream_on = true;
  Serial.println(F("data stream on"));
}


void CmdStreamOff() {
  stream_on = false;
  Serial.println(F("data stream off"));
}


void CmdTest() {
  Serial.println(F("test"));
}


void CmdDebugOn() {
  debug_on = true;
  radio.DebugOn();
  Serial.println(F("debugging enabled"));
}


void CmdDebugOff() {
  debug_on = false;
  radio.DebugOff();
  Serial.println(F("debugging disabled"));
}


void CmdReadReg() {
  char *arg = scmd.next();
  if (arg != NULL) {
    Serial.println(radio.ReadReg(atoi(arg)), HEX);
  }
}


void CmdWriteReg() {
  char *arg1 = scmd.next();
  char *arg2 = scmd.next();
  if (arg1 != NULL or arg2 != NULL) {
    uint8_t address = atoi(arg1);
    uint8_t value = atoi(arg2);
    radio.WriteReg(address, value);
  }
}


void CmdStrobe() {
  char *arg = scmd.next();
  if (arg != NULL) {
    radio.SendStrobe(atoi(arg));
  }
}


// From http://jeelabs.org/2011/05/22/atmega-memory-use/
void CmdFreeRam() {
  extern int __heap_start, *__brkval;
  int v;
  Serial.print(F("free mem: "));
  Serial.println(
      (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
}


void CmdUnrecognized(const char *cmd) {
  Serial.print(F("error: unknown command: "));
  Serial.println(cmd);
}


void CmdNull() {}


