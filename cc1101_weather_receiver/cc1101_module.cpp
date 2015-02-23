// Copyright (c) 2014 Chris Matteri
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
      channel_set_(false),
      state_(kIdle),
      packet_received_(false),
      packet_ready_for_read_(false),
      synchronize_channel_(0),
      frequency_table_length_(51),
      debug_on_(false) {}


void CC1101Module::Initialize(const bool activeIds[], byte region) {
  // These periods were determined empirically.
  weather_devices_[0].transmission_period = 2563091;
  weather_devices_[1].transmission_period = 2625609;
  weather_devices_[2].transmission_period = 2688137;
  weather_devices_[3].transmission_period = 2750612;
  weather_devices_[4].transmission_period = 2813176;
  weather_devices_[5].transmission_period = 2875647;
  weather_devices_[6].transmission_period = 2938151;
  weather_devices_[7].transmission_period = 3000658;

  for (byte i = 0; i < kNumIds; i++) {
    weather_devices_[i].id = i + 1;
  }

  pinMode(SS, OUTPUT);

  // Depending on jumper settings on the CC1101-CC1190 EM, these outputs control
  // the CC1190.
  // High gain mode and the low noise amplifier are enabled.  The power
  // amplifier is only used for transmission, and this device does not transmit,
  // so it is disabled.
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
  // indicating that the crystal oscillator has stabilized.  If the Arduino
  // was not reset by power-on, the CC1101 will not automatically reset
  // itself, so we send the SRES strobe to reset it.
  noInterrupts();
  uint32_t start_time = millis();
  digitalWrite(SS, LOW);
  while(digitalRead(MISO)) {  // Wait for XOSC Stable.
    // millis() will not overflow since a power-on/reset just occurred.
    if(millis() - start_time > 5) {
      Serial.println("error: CC1101 is not responding.");
      return;
    }
  }
  SPI.transfer(kSRES);
  while(digitalRead(MISO)) {  // Wait for reset to complete.
    if(millis() - start_time > 5) {
       Serial.println("error: resetting CC1101 failed.");
       return;
    }
  }
  digitalWrite(SS, HIGH);
  interrupts();

  const uint8_t rfSettings[][2] = {
      // GDO2 Output Pin Configuration
      // The Rx FIFO threshold is kept at the default value of 32, which is
      // never reached due to a packet length of 8, so GDO2 will go high at
      // end of packet.
      { kIOCFG2, 0x01 },

      // GDO0 Output Pin Configuration
      { kIOCFG0, 0x06 },

      // RX FIFO and TX FIFO Thresholds
      // TODO: comment.
      //{ kFIFOTHR, 0x41 },

      // Sync Word, High Byte
      { kSYNC1, 0xCB },

      // Sync Word, Low Byte
      { kSYNC0, 0x89 },

      // Packet Length
      // Davis uses 8 data bytes.
      // TODO: fix
      { kPKTLEN, 0x08 },

      // Packet Automation Control
      // Preamble quality threshold = 16.
      // CRC_AUTOFLUSH = 0. APPEND_STATUS = 0.  No address check.
      { kPKTCTRL1, 0x80 },

      // Packet Automation Control
      // Whitening off.  Use FIFOs for RX and TX data.
      // CRC_EN = 0.  Fixed packet length mode.
      { kPKTCTRL0, 0x00 },

      // Device Address
      { kADDR, 0x00 },

      // Channel Number
      { kCHANNR, 0x00 },

      // Frequency Synthesizer Control
      // IF is 152 kHz.  This value is from SmartRF Studio.
      { kFSCTRL1, 0x06 },

      // Frequency Synthesizer Control
      // Frequency offset is 0.
      { kFSCTRL0, 0x00 },

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
      // 9.6 kHz deviation.
      { kDEVIATN, 0x24 },

      // Main Radio Control State Machine Configuration
      // RX_TIME_RSSI = 0.  RX_TIME_QUAL = 0.
      // No RX timeout.
      // TODO:  Implement an RX timeout.
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

  state_ = kSynchronizing;
  SetChannel(synchronize_channel_);

  instancePointer = this;
}


void CC1101Module::Update() {
  noInterrupts();
  if (channel_set_) {
    channel_set_ = false;
    if (debug_on_) {
      Serial.write('C');
      Serial.println(channel_);
    }
  }
  interrupts();
  if (packet_received_) {
    packet_received_ = false;
    noInterrupts();
    ProcessPacket();
    interrupts();
  }
  if (state_ == kIdle) {
    NewTask();
  }
}


const byte *CC1101Module::packet() {
  packet_ready_for_read_ = false;
  return packet_;
}


byte CC1101Module::channel() {
  return channel_;
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
  noInterrupts();
  StopTimer(); // Disable timeout interrupt.
  receive_time_ = micros();
  packet_received_ = true;
  interrupts();
}


void CC1101Module::PacketArrivingInterruptHandler() {
  noInterrupts();
  StopTimer();
  if (digitalRead(kGDO0) || digitalRead(kGDO2)) {
    return;
  } else {
    ReceivePacket(*next_device_);
  }
  interrupts();
}


void CC1101Module::ReceiveTimeoutInterruptHandler() {
  noInterrupts();
  StopTimer();
  if (digitalRead(kGDO0) || digitalRead(kGDO2)) {
      return;
  } else {
    state_ = kIdle;
  }
  interrupts();
}


void CC1101Module::SetChannel(byte channel) {
  // If radio is not in IDLE state, set it to IDLE state.
  if (ReadReg(kMARCSTATE) != 1)
    SendStrobe(kSIDLE);

  channel_set_ = true;
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


// From http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
byte CC1101Module::ReverseBits(byte b) {
  b = ((b & 0b11110000) >> 4) | ((b & 0b00001111) << 4);
  b = ((b & 0b11001100) >> 2) | ((b & 0b00110011) << 2);
  b = ((b & 0b10101010) >> 1) | ((b & 0b01010101) << 1);
  return (b);
}


void CC1101Module::StartTimer(uint32_t period, void (*isr)()) {
  // The order of these function calls matters.
  // The TimerOne library operates timer 1 in phase and frequency correct PWM
  // mode.  In this mode the timer counts up to a value determined by ICR1,
  // then down to 0.  It appears that the register that determines whether the
  // timer is counting up or down is reset to up when WGM1[3:0] (thats WGM1
  // bits 3:0) is set to 0, which is done by Timer1.start().  Therefore, calling
  // Timer1.start() will restart the timer.  The value of the timer, TCNT1 is
  // set to 1 before attaching an interrupt so that TOV1 does not immediately
  // become set.  The amount of error this adds to the timer period is
  // negligible for this application.  Before attaching an interrupt, 1 is
  // written to TIFR1 to reset TOV1, which is set by Timer1.start().
  Timer1.setPeriod(period);
  Timer1.start();
  TCNT1 = 1;
  TIFR1 = 1;
  Timer1.attachInterrupt(isr);
}


void CC1101Module::StopTimer() {
  Timer1.detachInterrupt();
}


void CC1101Module::ReceivePacket(volatile WeatherDevice &device) {
  state_ = kReceivingPacket;
  current_device_ = &device;

  SetChannel(device.next_packet_channel);
  uint32_t timeout_period = device.next_channel_switch_time + kReceiveTime + 2000
      - micros();
  StartTimer(timeout_period, CC1101Module::ReceiveTimeoutISR);
}


void CC1101Module::ProcessPacket() {
  digitalWrite(SS, LOW);
  SPI.transfer(
      kFIFO | kSPIHeaderRWBit | kSPIHeaderBurstBit);
  // TODO: check for empty somewhere here.  If you read the FIFO while empty it will underflow.
  for (byte i = 0; i < kPacketSize; i++) {
    packet_[i] = ReverseBits(SPI.transfer(0));
  }
  digitalWrite(SS, HIGH);
  uint16_t calculatedCRC = CRC16_CCITT(packet_, 6);
  if (calculatedCRC == word(packet_[6], packet_[7])) {
    // The number in the lowest three bits of byte 0 is one less than the
    // transmitter ID.
    byte id = (packet_[0] & 7) + 1;
    volatile WeatherDevice &device = weather_devices_[id - 1];
    if(debug_on_) {
      Serial.write('I');
      Serial.print(id);

      byte rssi_raw = ReadReg(kRSSI);
      int8_t rssi = *(int8_t *)&rssi_raw/2 - 74;
      Serial.write(' ');
      Serial.print(rssi);

      byte freq_est_raw = ReadReg(kFREQEST);
      int8_t freq_est = *(int8_t *)&freq_est_raw;
      Serial.write(' ');
      Serial.println(freq_est);
    }

    // TODO: Explain this if statement's purpose.
    if ((state_ == kReceivingPacket && device.id == current_device_->id)
        || (device.is_active && !device.is_synchronized)) {
      packet_ready_for_read_ = true;
      // This calculation will overflow eventually.  The overflow is taken
      // into account in Update().
      device.next_channel_switch_time = receive_time_ + device.transmission_period
          - kReceiveTime;
      device.next_packet_channel = channel_ + 1;
      if (device.next_packet_channel >= frequency_table_length_) {
        device.next_packet_channel = 0;
      }

      device.consecutive_missed = 0;
      device.packets_received++;

      if (device.is_synchronized == false) {
        device.is_synchronized = true;
        if (device.consecutive_missed > 0) {
          // Update packet stats for this device.
          device.num_resyncs++;
        }
      }
    } else if (state_ == kSynchronizing) {
      // If a Davis device is present but its ID isn't active, its packets
      // will still be received while synchronizing.  If the receiver
      // starts listening again on the same channel, it will often get a
      // CRC error from the trailing end of the transmission, causing
      // synchronize_channel_ to be incremented.  The receiver could start
      // following this inactive ID through the frequency hopping
      // sequence.  Decrementing synchronize channel here prevents this
      // issue.
      synchronize_channel_--;
      if (synchronize_channel_ > frequency_table_length_) {
        synchronize_channel_ = frequency_table_length_ - 1;
      }
    }
  } else {
    if (debug_on_) {
      Serial.println(F("Bad CRC"));
    }
    if (state_ == kSynchronizing) {
      // If we're synchronizing and we miss a packet, increment the
      // channel to receive the next one.
      synchronize_channel_++;
      if (synchronize_channel_ >= frequency_table_length_) {
        synchronize_channel_ = 0;
      }
    }

  }
  state_ = kIdle;
}


void CC1101Module::MissedPackets(volatile WeatherDevice &device) {
  if (debug_on_) {
    Serial.write('M');
    Serial.println(device.id);
  }

  uint32_t time_micros = micros();
  byte packets_missed = 0;
  while(device.next_channel_switch_time <= time_micros
      and time_micros - device.next_channel_switch_time < 1e8) {
    device.next_channel_switch_time += device.transmission_period;
    packets_missed++;
  }

  device.next_packet_channel += packets_missed;
  if (device.next_packet_channel >= frequency_table_length_) {
    device.next_packet_channel = 0;
  }

  device.consecutive_missed += packets_missed;
  device.packets_missed += packets_missed;

  if (device.consecutive_missed > kMaxConsecutiveMissed) {
    Serial.print(F("error: lost track of id: "));
    Serial.print(device.id);
    Serial.println(F("."));
    device.is_synchronized = false;
    return;
  }
}


void CC1101Module::Synchronize() {
  state_ = kSynchronizing;
  SetChannel(synchronize_channel_);
}


// TODO: The receiver should not listen for packets when none are expected, as
// this runs the risk of receiving an erroneous packet due to noise or
// interference.
void CC1101Module::NewTask() {
  uint32_t next_channel_switch_time;
  uint32_t time_micros = micros();
  next_device_ = NULL;

  for (byte i = 0; i < kNumIds; i++) {
    volatile WeatherDevice &device = weather_devices_[i];
    if (device.is_active == false || device.is_synchronized == false)
      continue;

    // We want to determine whether device.next_channel_switch_time represents
    // an earlier (or equal) time than time_micros.  The extra comparisons are
    // to account for the fact that micros() will eventually overflow.
    if ((device.next_channel_switch_time <= time_micros
        and time_micros - device.next_channel_switch_time < 1e8)
        or (device.next_channel_switch_time > time_micros
            and device.next_channel_switch_time - time_micros > 1e8)) {
      MissedPackets(device);
    }

    // Similarly here we seek to determine whether device.next_channel_switch_time
    // represents an earlier time than next_channel_switch_time;
    if (next_device_ == NULL
        or (device.next_channel_switch_time < next_channel_switch_time
            and next_channel_switch_time - device.next_channel_switch_time < 1e8)
        or (device.next_channel_switch_time > next_channel_switch_time
            and device.next_channel_switch_time - next_channel_switch_time > 1e8)) {
      next_channel_switch_time = device.next_channel_switch_time;
      next_device_ = &device;
    }
  }

  if (next_device_ == NULL) {
    // There are no synchronized devices.
    Synchronize();
    return;
  }

  // Check for unsynchronized devices.
  bool all_devices_are_synchronized = true;
  for (byte i = 0; i < kNumIds; i++) {
    volatile WeatherDevice &device = weather_devices_[i];
    if (device.is_active && device.is_synchronized == false) {
      all_devices_are_synchronized = false;
    }
  }

  // If a packet is scheduled to arrive soon, start listening for that packet
  // now.  Otherwise, set a timer to receive the next packet from a synchronized
  // device and start synchronizing.
  uint32_t delta_t;
  if (time_micros < next_channel_switch_time) {
    delta_t = next_channel_switch_time - time_micros;
  } else {
  //          <--     UINT32_MAX    -->
    delta_t = (uint32_t)0 - (uint32_t)1 - time_micros + next_channel_switch_time;
  }
  if (delta_t < 1000) {
    ReceivePacket(*next_device_);
  } else {
    StartTimer(delta_t, CC1101Module::PacketArrivingISR);
    if (all_devices_are_synchronized) {
      state_ = kWaitingForPacket;
    } else {
      Synchronize();
    }
  }
}

}  // namespace cc1101_weather_receiver
