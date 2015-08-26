// Copyright (c) 2014-2015 Chris Matteri
// Released under the MIT License (http://opensource.org/licenses/mit-license.php)

#include "cc1101_module.h"

namespace cc1101_weather_receiver {

const uint8_t CC1101Module::freqs_us[51][3] = {
    {0X23, 0x0D, 0xEC},
    {0X22, 0xB4, 0xFE},
    {0X23, 0x12, 0xDB},
    {0X23, 0x7F, 0x8A},
    {0X23, 0x30, 0x82},
    {0X22, 0xDC, 0x84},
    {0X23, 0x9D, 0x31},
    {0X23, 0x53, 0x16},
    {0X22, 0xF5, 0x37},
    {0X23, 0x66, 0xD9},
    {0X23, 0x21, 0xAE},
    {0X22, 0xC3, 0xD2},
    {0X23, 0x44, 0x45},
    {0X23, 0x8E, 0x5D},
    {0X23, 0x04, 0x09},
    {0X22, 0xCD, 0xB1},
    {0X23, 0x3A, 0x61},
    {0X23, 0x70, 0xB9},
    {0X22, 0xE6, 0x65},
    {0X23, 0xA7, 0x11},
    {0X23, 0x1C, 0xBE},
    {0X22, 0xBE, 0xDF},
    {0X23, 0x49, 0x33},
    {0X23, 0x84, 0x7E},
    {0X22, 0xFA, 0x29},
    {0X23, 0xA2, 0x21},
    {0X22, 0xD7, 0x94},
    {0X23, 0x2B, 0x90},
    {0X23, 0x5C, 0xF6},
    {0X23, 0x93, 0x4D},
    {0X22, 0xB9, 0xF1},
    {0X23, 0x08, 0xFB},
    {0X23, 0x75, 0xA9},
    {0X23, 0x35, 0x72},
    {0X22, 0xE1, 0x75},
    {0X23, 0x4E, 0x23},
    {0X23, 0xAC, 0x03},
    {0X23, 0x6B, 0xC9},
    {0X22, 0xF0, 0x46},
    {0X23, 0x17, 0xCD},
    {0X23, 0x58, 0x07},
    {0X22, 0xC8, 0xC2},
    {0X23, 0x89, 0x6D},
    {0X23, 0x3F, 0x54},
    {0X22, 0xFF, 0x1B},
    {0X23, 0x61, 0xE8},
    {0X22, 0xD2, 0xA1},
    {0X23, 0x7A, 0x9B},
    {0X22, 0xEB, 0x56},
    {0X23, 0x26, 0x9F},
    {0X23, 0x98, 0x3E}
};

CC1101Module* CC1101Module::instancePointer;

CC1101Module::CC1101Module()
    : channel_(-1),
      receiving_packet_(false),
      searching_for_wx_devices_(false),
      receiving_from_tracked_device_(false),
      receive_timer_on_(false),
      packet_received_(false),
      packet_ready_for_read_(false),
      search_channel_(0),
      last_search_channel_change_time_(0),
      frequency_table_length_(51),
      debug_on_(false) {}

void CC1101Module::Initialize(const bool activeIds[], byte region) {
  for (byte i = 0; i < kNumIds; i++) {
    weather_devices_[i].id = i + 1;
    weather_devices_[i].transmission_period = 1e6 * (41 + i) / 16;
  }

  pinMode(MISO, INPUT_PULLUP);

  // Depending on jumper settings on the CC1101-CC1190 EM, these outputs
  // control the CC1190.  High gain mode and the low noise amplifier are
  // enabled.  The power amplifier is only used for transmission, and this
  // device does not transmit, so it is disabled.
  pinMode(kHGM, OUTPUT);
  pinMode(kLNA_EN, OUTPUT);
  pinMode(kPA_EN, OUTPUT);
  digitalWrite(kHGM, HIGH);
  digitalWrite(kLNA_EN, HIGH);
  digitalWrite(kPA_EN, LOW);

  // See the comment in StartTimer for how the Timer1 library is used in this
  // device.
  Timer1.initialize();

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  // The CC1101 includes an automatic power-on reset circuit.  See section
  // 19.1.1 of the datasheet for details.  Here we wait for MISO to go low,
  // indicating that the crystal oscillator has stabilized.  
  uint32_t start_time = millis();
  digitalWrite(SS, LOW);
  while(digitalRead(MISO)) {  // Wait for XOSC Stable.
    // millis() will not overflow since a power-on/reset just occurred.
    if(millis() - start_time > 5) {
      Serial.println("error: CC1101 is not responding.");
      return;
    }
  }

  // If the Arduino was not reset by power-on, the CC1101 will not
  // automatically reset itself. We reset so that if the user messed up any
  // registers with the debug commands they will be set to their defaults.
  ResetCC1101();

  const uint8_t rfSettings[][2] = {
      // GDO2 Output Pin Configuration
      // The Rx FIFO threshold is kept at the default value of 32, which is
      // never reached due to a packet length of 8, so GDO2 will go high at
      // end of packet.
      { kIOCFG2, 0x01 },

      // GDO0 Output Pin Configuration
      // Set when the sync word has been received. Cleared at end of packet
      // (and several other conditions not relevant here).
      { kIOCFG0, 0x06 },

      // Sync Word, High Byte
      { kSYNC1, 0xCB },

      // Sync Word, Low Byte
      { kSYNC0, 0x89 },

      // Packet Length
      // Davis uses 10 data bytes (not including preamble or sync bytes).
      { kPKTLEN, kPacketSize },

      // Packet Automation Control
      // Preamble quality threshold = 16.
      // CRC_AUTOFLUSH = 0. APPEND_STATUS = 0.  No address check.
      { kPKTCTRL1, 0x80 },

      // Packet Automation Control
      // Whitening off.  Use FIFOs for RX and TX data.
      // CRC_EN = 0.  Fixed packet length mode.
      { kPKTCTRL0, 0x00 },

      // Frequency Synthesizer Control
      // IF is 152 kHz.  This value is from SmartRF Studio.
      { kFSCTRL1, 0x06 },

      // Frequency Synthesizer Control
      // Frequency offset is 0.
      { kFSCTRL0, 0 },

      // Modem Configuration
      // Channel bandwidth is 101.6 kHz.
      // Data rate is 19192 Hz (target 19.2 kHz).
      { kMDMCFG4, 0xC9 },

      // Modem Configuration
      // Data rate is 19192 Hz (target 19.2 kHz).
      { kMDMCFG3, 0x83 },

      // Modem Configuration
      // Enable DC blocking filter.  GFSK Modulation.
      // Manchester encoding off.  Need 16/16 bits correct in sync word
      // to begin packet reception.
      { kMDMCFG2, 0x12 },

      // Modem Configuration
      // No forward error correction.  4 preamble bytes (for TX I believe).
      // 199.951 kHz channel spacing (not used for selecting frequency).
      { kMDMCFG1, 0x22 },

      // Modem Deviation Setting
      // 9.5 kHz deviation.
      { kDEVIATN, 0x24 },

      // Main Radio Control State Machine Configuration
      // RX_TIME_RSSI = 0.  RX_TIME_QUAL = 0.
      // No RX timeout (that's implemented here).
      { kMCSM2, 0x07 },

      // Main Radio Control State Machine Configuration
      // CCA_MODE = 3 (clear channel indication behavior).
      // Switch to IDLE mode after receiving a packet.
      { kMCSM1, 0x30 },

      // Main Radio Control State Machine Configuration
      // Calibrate when switching from IDLE to RX, TX or FSTXON.
      // PO_TIMEOUT = 2.  PIN_CTRL_EN = 0.  XOSC_FORCE_ON = 0.
      { kMCSM0, 0x18 },

      // Frequency Offset Compensation Configuration
      // FOC_BS_CS_GATE = 1.  FOC_PRE_K = 2.  FOC_POST_K = 1.
      // FOC_LIMIT = 2.
      { kFOCCFG, 0x36 },

      // Frequency Synthesizer Calibration
      // Value from SmartRF Studio.
      { kFSCAL3, 0xE9 },

      // Frequency Synthesizer Calibration
      // Value from SmartRF Studio.
      { kFSCAL2, 0x2A },

      // Frequency Synthesizer Calibration
      // Value from SmartRF Studio.
      { kFSCAL1, 0x00 },

      // Frequency Synthesizer Calibration
      // Value from SmartRF Studio.
      { kFSCAL0, 0x1F },

      // Various Test Settings
      // Value from SmartRF Studio.
      { kTEST2, 0x81 },

      // Various Test Settings
      // Value from SmartRF Studio.
      { kTEST1, 0x35 },

      // Various Test Settings
      // Value from SmartRF Studio.
      { kTEST0, 0x09 } };

  const byte rf_settings_size = sizeof(rfSettings) / 2;
  for (byte i = 0; i < rf_settings_size; i++) {
    WriteReg(rfSettings[i][0], rfSettings[i][1]);
  }

  // Interrupt 1 corresponds to pin 3, with is connected to GDO2 on the CC1101.
  attachInterrupt(1, PacketReceivedISR, RISING);

  // Set which transmitter IDs the receiver will listen for.
  for (byte i = 0; i < kNumIds; i++)
    weather_devices_[i].is_active = activeIds[i];

  SearchForWXDevices();
  instancePointer = this;
}

void CC1101Module::Loop() {
  if (receiving_packet_) {
    receiving_packet_ = false;
    if (debug_on_) {
      Serial.write('C');
      Serial.print(channel_);
      Serial.write(' ');
    }
  }
  if (packet_received_) {
    packet_received_ = false;
    ProcessPacket();
  }
  if (!searching_for_wx_devices_ && !receiving_from_tracked_device_ 
      && !receive_timer_on_) {
    NewTask();
  }
  if (millis() - last_search_channel_change_time_ > 120000) {
    search_channel_++;
    if (search_channel_ >= frequency_table_length_) search_channel_ = 0;
    last_search_channel_change_time_ = millis();
    noInterrupts();
    if (searching_for_wx_devices_ && !receive_timer_on_) SearchForWXDevices();
    interrupts();
  }
}

const byte *CC1101Module::packet() {
  if (packet_ready_for_read_) {
    packet_ready_for_read_ = false;
    return packet_;
  }
  return NULL;
}

byte CC1101Module::ReadReg(byte addr) {
  noInterrupts();
  digitalWrite(SS, LOW);
  cc1101_status_byte_ = SPI.transfer(addr | kSPIHeaderRWBit);
  byte regval = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  interrupts();
  return regval;
}

void CC1101Module::WriteReg(byte addr, byte value) {
  noInterrupts();
  digitalWrite(SS, LOW);
  cc1101_status_byte_ = SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(SS, HIGH);
  interrupts();
}

void CC1101Module::SendStrobe(byte strobe) {
  noInterrupts();
  digitalWrite(SS, LOW);
  cc1101_status_byte_ = SPI.transfer(strobe);
  digitalWrite(SS, HIGH);
  interrupts();
}

void CC1101Module::SendStrobeWithReadBit(byte strobe) {
  noInterrupts();
  digitalWrite(SS, LOW);
  cc1101_status_byte_ = SPI.transfer(strobe | kSPIHeaderRWBit);
  digitalWrite(SS, HIGH);
  interrupts();
}

void CC1101Module::PacketReceivedISR() {
  instancePointer->PacketReceivedInterruptHandler();
}

void CC1101Module::PacketArrivingISR() {
  instancePointer->PacketArrivingInterruptHandler();
}

void CC1101Module::ReceiveTimeoutISR() {
  instancePointer->ReceiveTimeoutInterruptHandler();
}

void CC1101Module::PacketReceivedInterruptHandler() {
  StopTimer(); // Disable timeout interrupt.
  receive_time_ = micros();
  packet_received_ = true;
}

void CC1101Module::PacketArrivingInterruptHandler() {
  StopTimer();
  ReceiveFromDevice();
}

void CC1101Module::ReceiveTimeoutInterruptHandler() {
  StopTimer();
  receiving_from_tracked_device_ = false;
  SendStrobe(kSIDLE);
}

bool CC1101Module::Time1IsEarlier(uint32_t time1, uint32_t time2) {
  if (time1 == time2) return false;
  return time2 - time1 < UINT32_MAX/2 + 1;
}

void CC1101Module::ReceivePacket(byte channel) {
  // If radio is not in IDLE state, set it to IDLE state.
  if (ReadReg(kMARCSTATE) != 1)
    SendStrobe(kSIDLE);
  receiving_packet_ = true;
  if (channel_ != channel) {
    channel_ = channel;
    WriteReg(kFREQ2, freqs_us[channel_][0]);
    WriteReg(kFREQ1, freqs_us[channel_][1]);
    WriteReg(kFREQ0, freqs_us[channel_][2]);
  }
  // Clear the RX FIFO in case it was somehow underflowed.
  SendStrobe(kSFRX);
  SendStrobe(kSRX);
}

void CC1101Module::ResetCC1101() {
  uint32_t start_time = millis();
  noInterrupts();
  SPI.transfer(kSRES);
  interrupts();
  while(digitalRead(MISO)) {  // Wait for reset to complete.
    uint32_t time_millis = millis();
    if (time_millis - start_time > 5) {
       Serial.println("error: resetting CC1101 failed.");
       while (true);
    }
  }
  digitalWrite(SS, HIGH);
}

// From http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
byte CC1101Module::ReverseBits(byte b) {
  b = ((b & 0b11110000) >> 4) | ((b & 0b00001111) << 4);
  b = ((b & 0b11001100) >> 2) | ((b & 0b00110011) << 2);
  b = ((b & 0b10101010) >> 1) | ((b & 0b01010101) << 1);
  return (b);
}

void CC1101Module::StartTimer(uint32_t period, void (*isr)()) {
  Timer1.setPeriod(period);
  Timer1.start();  // This gets the timer counting up.
  Timer1.stop();  // Best to have the timer stopped until
                  // attachInterrupt starts it.
  TCNT1 = 1;  // Set to 1 to prevent TOV1 flag from being set. Error due to
              // shortening period is negligible in this application.
  TIFR1 = 1;  // Reset TOV1 flag.
  Timer1.attachInterrupt(isr);  // Attach interrupt and resume.
}

void CC1101Module::StopTimer() {
  receive_timer_on_ = false;
  Timer1.detachInterrupt();
}

void CC1101Module::ReceiveFromDevice() {
  searching_for_wx_devices_ = false;
  receiving_from_tracked_device_ = true;
  ReceivePacket(next_packet_channel_);
  StartTimer(kReceiveTime + kTimeoutTime, CC1101Module::ReceiveTimeoutISR);
}

bool CC1101Module::CheckCRC(uint8_t *packet) {
  uint8_t buf[8];
  memcpy(buf, packet, 6);
  uint16_t calculated_crc;
  if (packet[8] == 0xff and packet[9] == 0xff) {
    calculated_crc = CRC16_CCITT(buf, 6);
  } else {
    // The packet came from a repeater, so bytes 8 and 9 need to be included in
    // the CRC calculation.
    buf[6] = packet[8];
    buf[7] = packet[9];
    calculated_crc = CRC16_CCITT(buf, 8);
  }
  return calculated_crc == word(packet[6], packet[7]);
}

void CC1101Module::ReadPacket() {
  digitalWrite(SS, LOW);
  SPI.transfer(
      kFIFO | kSPIHeaderRWBit | kSPIHeaderBurstBit);
  for (byte i = 0; i < kPacketSize; i++) {
    packet_[i] = ReverseBits(SPI.transfer(0));
  }
  digitalWrite(SS, HIGH);
}

void CC1101Module::PrintPacketRadioStats() {
  Serial.write('R');
  byte rssi_raw = ReadReg(kRSSI);
  int8_t rssi = *(int8_t *)&rssi_raw/2 - 74;
  Serial.print(rssi);
  byte freq_est_raw = ReadReg(kFREQEST);
  int8_t freq_est = *(int8_t *)&freq_est_raw;
  Serial.write(' ');
  Serial.print(freq_est);
  Serial.write(' ');
}

void CC1101Module::PrintPacketIDAndRepeaterBytes() {
  Serial.write('I');
  Serial.print((packet_[0] & 7) + 1);
  Serial.write(' ');
  Serial.print(packet_[8], HEX);
  Serial.write(' ');
  Serial.println(packet_[9], HEX);
}

void CC1101Module::UpdateWXDevice(WeatherDevice &device) {
  device.next_channel_switch_time = receive_time_ + device.transmission_period
                                    - kReceiveTime;
  device.next_packet_channel = channel_ + 1;
  if (device.next_packet_channel >= frequency_table_length_) {
    device.next_packet_channel = 0;
  }
  device.consecutive_missed = 0;
  device.is_tracked = true;
}

void CC1101Module::ProcessPacket() {
  ReadPacket();
  if (debug_on_) PrintPacketRadioStats();
  if (CheckCRC(packet_)) {
    // The number in the lowest three bits of byte 0 is one less than the
    // transmitter ID.
    byte id = (packet_[0] & 7) + 1;
    WeatherDevice &device = weather_devices_[id - 1];
    if(debug_on_) PrintPacketIDAndRepeaterBytes();
    if ((receiving_from_tracked_device_ && device.id == next_device_->id)
        || (device.is_active && !device.is_tracked)) {
      packet_ready_for_read_ = true;
      UpdateWXDevice(device);
    } else if (searching_for_wx_devices_) {
      // If a Davis device is present but its ID isn't active, its packets will
      // still be received while searching. If the receiver starts listening
      // again on the same channel, it will often get a CRC error from the
      // trailing end of the transmission, causing search_channel_ to be
      // incremented. The receiver could start following this inactive ID
      // through the frequency hopping sequence. Decrementing the search
      // channel prevents this issue.
      search_channel_--;
      last_search_channel_change_time_ = millis();
      if (search_channel_ > frequency_table_length_) {
        search_channel_ = frequency_table_length_ - 1;
      }
    }
  } else {
    if (debug_on_) Serial.println(F("Bad CRC"));
    if (searching_for_wx_devices_) {
      // If the CRC error was caused by a transmission from a weather device we
      // want to track, we can attempt to receive the next packet from that
      // device by incrementing the channel.
      search_channel_++;
      last_search_channel_change_time_ = millis();
      if (search_channel_ >= frequency_table_length_) search_channel_ = 0;
    }
  }
  searching_for_wx_devices_ = false;
  receiving_from_tracked_device_ = false;
}

void CC1101Module::MissedPacket(WeatherDevice &device) {
  if (debug_on_) {
    Serial.write('M');
    Serial.println(device.id);
  }
  device.next_channel_switch_time += device.transmission_period;
  device.next_packet_channel++;
  if (device.next_packet_channel == frequency_table_length_) {
      device.next_packet_channel = 0;
  }
  device.consecutive_missed++;

  if (device.consecutive_missed > kMaxConsecutiveMissed) {
    Serial.print(F("error: lost track of id: "));
    Serial.print(device.id);
    Serial.println(F("."));
    device.is_tracked = false;
    return;
  }
}

void CC1101Module::SearchForWXDevices() {
  searching_for_wx_devices_ = true; 
  ReceivePacket(search_channel_);
}

bool CC1101Module::AllWXDevicesAreTracked() {
  for (byte i = 0; i < kNumIds; i++) {
    WeatherDevice &device = weather_devices_[i];
    if (device.is_active && device.is_tracked == false) {
      return false;
    }
  }
  return true;
}

void CC1101Module::NewTask() {
  uint32_t next_channel_switch_time;
  uint32_t time_micros = micros();
  next_device_ = NULL;

  for (byte i = 0; i < kNumIds; i++) {
    WeatherDevice &device = weather_devices_[i];
    if (!device.is_active || !device.is_tracked) continue;

    while (Time1IsEarlier(device.next_channel_switch_time, time_micros)) {
      MissedPacket(device);
      // MissedPacket can set a device to be untracked, so we need to check
      // again.
      if (!device.is_tracked) continue;
    }

    if (next_device_ == NULL || Time1IsEarlier(device.next_channel_switch_time,
        next_channel_switch_time)) {
      next_channel_switch_time = device.next_channel_switch_time;
      next_device_ = &device;
    }
  }

  if (next_device_ == NULL) {
    // No devices are being tracked.
    SearchForWXDevices();
    return;
  }

  next_packet_channel_ = next_device_->next_packet_channel;
  uint32_t delta_t = next_channel_switch_time - time_micros;
  if (!delta_t) delta_t = 1;  // Another hack to deal with TimerOne.
  if (delta_t > 1000 && !AllWXDevicesAreTracked()) SearchForWXDevices();
  receive_timer_on_ = true;
  StartTimer(delta_t, CC1101Module::PacketArrivingISR);
}

}  // namespace cc1101_weather_receiver
