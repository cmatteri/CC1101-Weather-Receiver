// Copyright (c) 2014 Chris Matteri
// Released under the MIT License (http://opensource.org/licenses/mit-license.php)

#ifndef CC1101_MODULE_H_
#define CC1101_MODULE_H_

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <TimerOne.h>

#include "crc.h"
#include "weather_device.h"

namespace cc1101_weather_receiver {

class CC1101Module {
public:
  enum Region {
    kUS,
    kEU,
    kNZ,
    kAU
  };

  static const byte kNumIds = 8;
  static const byte kPacketSize = 8;

  CC1101Module();
  void Initialize(const bool activeIds[], byte region);

  // Returns true if a new packet is ready to be read with packet().
  // Return false once the packet has been read.
  bool PacketReady() { return packet_ready_for_read_; }

  // Update should be called in loop().
  void Update();

  // Accessor for packet_.
  const byte *packet();

  // These are made public to facilitate debugging.
  byte channel();  // Accessor for channel_.
  byte ReadReg(byte addr);
  void WriteReg(byte addr, byte val);
  void SendStrobe(byte strobe);
  void SendStrobeWithReadBit(byte strobe);
  void DebugOn() { debug_on_ = true; }  // Enable printing of debug information.
  void DebugOff() { debug_on_ = false; }  // Disable printing of debug
                                          // information.

protected:
  enum State {
    kReceivingPacket, kSynchronizing, kIdle
  };

  // The maximum number of consecutive packets missed from a device before
  // the receiver attempts to resynchronize with it.
  static const uint8_t kMaxConsecutiveMissed = 10;

  // A packet has 4 preamble bytes, 2 sync bytes and 8 data bytes.  At a bitrate
  // of 19.2 kHz, receiving a packet will take
  // 14 * 8 * 1 / (19.2 kHz) = 5833 us.  However, tests show that the stations
  // start transmitting about 17 ms before then end of packet reception.  If
  // the channel change is made less than about 17 ms before the end of packet
  // reception, the packet is missed.  The time before the preamble bytes is
  // presumably needed for the receiver to calibrate the oscillator or some
  // similar action.  It's possible that the extra time is needed to account for
  // error in the transmitter period, either in my measurements or in the
  // oscillator on the transmitter. The receiver will prepare to receive a
  // packet kReceiveTime microseconds before the expected end of packet
  // reception.

  static const uint32_t kReceiveTime = 30000;  // microseconds
  static const uint8_t freqs_us[51][3];  // Holds the frequency hopping
                                         // sequence used by Davis weather
                                         // devices.  Three bytes are stored
                                         // for each frequency to program the
                                         // CC1101's three frequency registers.

  // Pins on the Arduino to which the CC1101-CC1190 Module is connected.
  static const byte kGDO0 = 2;
  static const byte kGDO2 = 3;
  static const byte kHGM = 4;
  static const byte kLNA_EN = 5;
  static const byte kPA_EN = 6;


  static const byte kSPIHeaderRWBit = 0x80;
  static const byte kSPIHeaderBurstBit = 0x40;

  // CC1101 Registers
  static const byte kIOCFG2 = 0x00;
  static const byte kIOCFG1 = 0x01;
  static const byte kIOCFG0 = 0x02;
  static const byte kFIFOTHR = 0x03;
  static const byte kSYNC1 = 0x04;
  static const byte kSYNC0 = 0x05;
  static const byte kPKTLEN = 0x06;
  static const byte kPKTCTRL1 = 0x07;
  static const byte kPKTCTRL0 = 0x08;
  static const byte kADDR = 0x09;
  static const byte kCHANNR = 0x0A;
  static const byte kFSCTRL1 = 0x0B;
  static const byte kFSCTRL0 = 0x0C;
  static const byte kFREQ2 = 0x0D;
  static const byte kFREQ1 = 0x0E;
  static const byte kFREQ0 = 0x0F;
  static const byte kMDMCFG4 = 0x10;
  static const byte kMDMCFG3 = 0x11;
  static const byte kMDMCFG2 = 0x12;
  static const byte kMDMCFG1 = 0x13;
  static const byte kMDMCFG0 = 0x14;
  static const byte kDEVIATN = 0x15;
  static const byte kMCSM2 = 0x16;
  static const byte kMCSM1 = 0x17;
  static const byte kMCSM0 = 0x18;
  static const byte kFOCCFG = 0x19;
  static const byte kBSCFG = 0x1A;
  static const byte kAGCCTRL2 = 0x1B;
  static const byte kAGCCTRL1 = 0x1C;
  static const byte kAGCCTRL0 = 0x1D;
  static const byte kWOREVT1 = 0x1E;
  static const byte kWOREVT0 = 0x1F;
  static const byte kWORCTRL = 0x20;
  static const byte kFREND1 = 0x21;
  static const byte kFREND0 = 0x22;
  static const byte kFSCAL3 = 0x23;
  static const byte kFSCAL2 = 0x24;
  static const byte kFSCAL1 = 0x25;
  static const byte kFSCAL0 = 0x26;
  static const byte kRCCTRL1 = 0x27;
  static const byte kRCCTRL0 = 0x28;
  static const byte kFSTEST = 0x29;
  static const byte kPTEST = 0x2A;
  static const byte kAGCTEST = 0x2B;
  static const byte kTEST2 = 0x2C;
  static const byte kTEST1 = 0x2D;
  static const byte kTEST0 = 0x2E;
  static const byte kFIFO = 0x3F;
  // Status Registers.
  // Same addresses as strobes, but with the burst bit set.  Read only.
  static const byte kPARTNUM = 0xF0;
  static const byte kVERSION = 0xF1;
  static const byte kFREQEST = 0xF2;
  static const byte kLQI = 0xF3;
  static const byte kRSSI = 0xF4;
  static const byte kMARCSTATE = 0xF5;
  static const byte kWORTIME1 = 0xF6;
  static const byte kWORTIME0 = 0xF7;
  static const byte kPKTSTATUS = 0xF8;
  static const byte kVCO_VC_DAC = 0xF9;
  static const byte kTXBYTES = 0xFA;
  static const byte kRXBYTES = 0xFB;
  static const byte kRCCTRL1_STATUS = 0xFC;
  static const byte kRCCTRL0_STATUS = 0xFD;
  // Command Strobes
  static const byte kSRES = 0x30;
  static const byte kSFSTXON = 0x31;
  static const byte kSXOFF = 0x32;
  static const byte kSCAL = 0x33;
  static const byte kSRX = 0x34;
  static const byte kSTX = 0x35;
  static const byte kSIDLE = 0x36;
  static const byte kSWOR = 0x38;
  static const byte kSPWD = 0x39;
  static const byte kSFRX = 0x3A;
  static const byte kSFTX = 0x3B;
  static const byte kSWORRST = 0x3C;
  static const byte kSNOP = 0x3D;

  // These static functions are necessary because interrupt functions cannot
  // belong to an object.  They use instancePointer to call the corresponding
  // method of the CC1101Module object.  Isn't C++ an elegant language for
  // programming microcontrollers?
    static void PacketReceivedISR();
  static void PacketArrivingISR();
  static void ReceiveTimeoutISR();
  // TODO: Are these virtual for a reason?
  void virtual PacketReceivedInterruptHandler();
  void virtual PacketArrivingInterruptHandler();
  void virtual ReceiveTimeoutInterruptHandler();
  void SetChannel(byte channel);  // channel is an index in freqs_us.
  byte ReverseBits(byte b);
  void StartTimer(uint32_t period, void (*isr)());
  void StopTimer();

  // Set the channel to receive the next packet from device.  Set a timeout
  // timer in case the packet does not arrive.
  void ReceivePacket(volatile WeatherDevice &device);

  // Called by Update() when after the CC1101 has received a packet.  Reads the
  // packet from the CC1101, performs a CRC check and updates the state of the
  // CC1101Module object based on the packet that arrived.
  void ProcessPacket();

  // Called on a device if one or more packets from that device were missed.
  // Will set the device to unsynchronized (i.e. the receiver will try to
  // resynchronize with that device) if more than kMaxConsecutiveMissed
  // consecutive packets are missed.
  void MissedPackets(volatile WeatherDevice &device);

  // Sets the receiver to listen for packets on synchronize_channel_.
  void Synchronize();

  // Find the active, synchronized device, if any, with the earlier packet
  // arrival time.  If there is no such device, start synchronizing.  If all
  // devices are synchronized, or the next scheduled packet is arriving soon,
  // set the channel to receive that packet.  Otherwise, set a timer to
  // receive that packet when it arrives, and start synchronizing.
  void NewTask();

  static CC1101Module* instancePointer;
  volatile byte channel_;
  volatile bool channel_set_; // SetChannel is called in interrupts, so
                              // it cannot use the Serial object.
                              // This flag is used to print channel changes
                              // in Update() when debugging is enabled.
  volatile byte cc1101_status_byte_;
  volatile uint32_t receive_time_;  // The time when a packet has been received
                                    // by the CC1101.

  bool packet_ready_for_read_;
  const byte frequency_table_length_;
  byte packet_[kPacketSize];  // Buffer to hold packets read from the CC1101.

  // A WeatherDevice object corresponds to a Davis weather device.  There are
  // 8 WeatherDevice objects for tracking up to 8 Davis devices.
  // Note: An ID is one higher than its index in ids, e.g., the device set to
  // ID 1 via the dipswitches in the device corresponds to weather_devices[0].
  volatile WeatherDevice weather_devices_[kNumIds];
  volatile byte state_;  // Used by the CC1101Module to track its current task.
                         // Not to be confused with the CC1101's internal state.
                         // Values should be elements of the State enum.
  volatile bool packet_received_;  // Used by the packet received interrupt to
                                   // signal that the CC1101 has received a
                                   // packet.
  volatile WeatherDevice *current_device_;  // When state_ is kReceivingPacket,
                                            // indicates the device the packet
                                            // is arriving from.
  volatile WeatherDevice *next_device_;  // When state_ is kSynchronizing and
                                         // a timer has been set to receive a
                                         // packet from a synchronized device,
                                         // points to that device.

  byte synchronize_channel_;  // The radio attempts to synchronize IDs on this
                              // channel.  It is incremented if a CRC error
                              // occurs while synchronizing.
  bool debug_on_;  // Determines whether to print debugging information.
};

}  // namespace cc1101_weather_receiver

#endif  // CC1101_MODULE_H_

