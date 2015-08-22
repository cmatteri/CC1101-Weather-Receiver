// Copyright (c) 2014-2015 Chris Matteri
// Released under the MIT License (http://opensource.org/licenses/mit-license.php)

#ifndef CC1101_MODULE_H_
#define CC1101_MODULE_H_

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <TimerOne.h>

#include "crc.h"
#include "weather_device.h"

// We don't have this in C++.
#define UINT32_MAX ((uint32_t)0 - (uint32_t)1)

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
  static const byte kPacketSize = 10;

  CC1101Module();
  void Initialize(const bool activeIds[], byte region);

  // Loop should be called in main loop.
  void Loop();

  // If a packet is ready to be read, a pointer to a buffer containing that
  // packet is returned. Otherwise NULL is returned. Only one call to packet
  // will return a non-Null pointer for each packet. The packet will never be
  // overwritten until Loop() is called.
  const byte *packet();

  // These are made public to facilitate debugging.
  byte channel() { return channel_; }
  byte ReadReg(byte addr);
  void WriteReg(byte addr, byte val);
  void SendStrobe(byte strobe);
  void SendStrobeWithReadBit(byte strobe);
  void DebugOn() { debug_on_ = true; }  // Enable printing of debug 
                                        // information.
  void DebugOff() { debug_on_ = false; }  // Disable printing of debug
                                          // information.

protected:
  // The maximum number of consecutive packets missed from a device before
  // the receiver attempts to resynchronize with it.
  static const uint8_t kMaxConsecutiveMissed = 10;

  // A packet has 4 preamble bytes, 2 sync bytes and 10 data bytes.  At a
  // bitrate of 19.2 kHz, receiving a packet will take 16 * 8 * 1 / (19.2 kHz)
  // = 6667 us. Switching the CC1101 from IDLE to RX with calibration requires
  // 800 us. With a crystal oscillator, the average error for packet arrival
  // time is less than 100 us. We want the receiver to work even if another
  // interfering Davis device is in range. The worst case for interference will
  // occur when the intefering device is just starting preamble byte 2 when the
  // radio is turned on, since with a PQI threshold of 4 it takes 2 bytes of
  // preamble before a sync word is accepted. If the correct transmitter starts
  // sending preamble bytes no later than the start of the second sync word of
  // the interfering transmitter, packets will not be received from the
  // interfering transmitter once the correct one is being tracked. If enough
  // packets are missed in a row, errors will accumulate and problems may occur
  // (only a problem for distant transmitters with weak signals). Compensating
  // for error in the transmitter periods may solve that problem, and may be
  // added at some point. Thus the radio should be turned on no sooner than 3
  // bytes = 1 / (19.2 kHz) * 24 = 1250 us before the expected arrival of the
  // first preamble byte. Thus kReceiveTime = 6667 + 800 + 1250 = 8717.
  static const uint32_t kReceiveTime = 8717;  // microseconds
  static const uint32_t kTimeoutTime = 2000;  // The amount of time to wait
                                              // after the expected packet
                                              // receipt time before triggering
                                              // a timeout.
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

  // The CC1101-CC1190 EM has a 26 MHz crystal.
  static const byte kCrystalFrequency = 26e6;
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
  // method of the CC1101Module object. Isn't C++ an elegant language for
  // programming microcontrollers?
  static void PacketReceivedISR();
  static void PacketArrivingISR();
  static void ReceiveTimeoutISR();
  void PacketReceivedInterruptHandler();
  void PacketArrivingInterruptHandler();
  void ReceiveTimeoutInterruptHandler();

  // Determines whether time1 represents an earlier time than time2. This
  // function uses modular arithmetic, so it is unaffected by overflow,
  // however, the maximum separation of the real times represented by the two
  // unsigned 32-bit integers must be less than (UINT32_MAX + 1) / 2 for this
  // function to return the correct value. This function only works for 32-bit
  // times, like those returned by Arduino's millis and micros functions. 
  bool Time1IsEarlier(uint32_t time1, uint32_t time2);
  void ReceivePacket(byte channel);  // channel is an index in freqs_us.
  void ResetCC1101();
  byte ReverseBits(byte b);
  // Wrappers the TimerOne library. Triggers an interrupt after a time
  // specified in us.
  void StartTimer(uint32_t period, void (*isr)());
  void StopTimer();

  // Set the channel to receive the next packet from a device.  Set a timeout
  // timer in case the packet does not arrive.
  void ReceiveFromDevice(WeatherDevice &device);

  // Check the packet CRC.  If bytes 8 and 9 are both 0xFF, only bytes 0
  // through 5 are used in the CRC calculation.  Otherwise, the CRC is
  // calculated on bytes 0 through 5, 8 and 9 (repeater packet). If the
  // calculated CRC matches that in bytes 6 and 7 of the packet, return 0,
  // otherwise return -1.
  bool CheckCRC(uint8_t *packet);

  // Read a received packet from the CC1101 into the packet_ buffer.
  void ReadPacket();

  // Prints the RSSI and frequency error (the difference between the
  // transmitter and receiver frequency) of the last packet received.
  void PrintPacketRadioStats();

  // Prints the transmitter ID and bytes 8 and 9 of the last packet received.
  void PrintPacketIDAndRepeaterBytes();

  // Called on a WeatherDevice object after a packet has been received from
  // that device. Updates the state of the device based on the new packet.
  void UpdateWXDevice(WeatherDevice &device);

  // Called by Loop() after the CC1101 has received a packet.  Reads the packet
  // from the CC1101, performs a CRC check and updates the state of the
  // CC1101Module object based on the packet that arrived.
  void ProcessPacket();

  // Called on a tracked device if a packet for that device was missed.  Will
  // set the device to untracked. If more than kMaxConsecutiveMissed
  // consecutive packets are missed, the device will be set to untracked (i.e.
  // the receiver will try to search for that device).
  void MissedPacket(WeatherDevice &device);

  // Search for untracked weather devices on search_channel_.
  void SearchForWXDevices();

  // Returns false if any active weather devices are not being tracked,
  // otherwise returns true.
  bool AllWXDevicesAreTracked();

  // Called after a packet has been received and the receiver must determine
  // what to do next. Find the active, tracked device, if any, with the
  // earliest packet arrival time in the future. If the packet arrival time for
  // any device is in the past, call MissedPacket on that device until it is in
  // the future. If there is no such device, start searching for devices.  If
  // the next scheduled packet is arriving soon, set the CC1101 to receive that
  // packet.  Otherwise, set a timer to receive that packet when it arrives,
  // and, if there are any untracked devices, search for them.
  void NewTask();

  static CC1101Module* instancePointer;
  volatile byte channel_;  // The channel (an index in freqs_us) that the
                           // CC1101 is set to.
  volatile bool receiving_packet_; // ReceivePacket is called in interrupts, so
                                   // it cannot use the Serial object.  This
                                   // flag is used to print attempts to receive
                                   // packets in Loop() when debugging is
                                   // enabled.
  volatile byte cc1101_status_byte_;  // Set whenever data is read from or 
                                      // written to the CC1101.
  volatile uint32_t receive_time_;  // The timestamp at the end of packet 
                                    // reception by the CC1101.

  bool packet_ready_for_read_;  // Flag to indicate whether a packet is ready 
                                // to be read out of the CC1101Module object.
  const byte frequency_table_length_;
  byte packet_[kPacketSize];  // Buffer to hold packets read from the CC1101.

  // A WeatherDevice object corresponds to a Davis weather device.  There are
  // 8 WeatherDevice objects for tracking up to 8 Davis devices.
  // Note: An ID is one higher than its index in ids, e.g., the device set to
  // ID 1 via the dipswitches in the device corresponds to weather_devices[0].
  WeatherDevice weather_devices_[kNumIds];
  bool searching_for_wx_devices_;  // Flag that is set when searching for 
                                   // untracked weather devices.
  bool receiving_from_tracked_device_;  // Flag that is set when receiving a
                                        // a packet from a tracked device, as
                                        // opposed to searching for devices.
  bool receive_timer_on_;  // Flag that is set when a timer is running that 
                           // will trigger an interrupt to receive a packet.
  volatile bool packet_received_;  // Used by the packet received interrupt to
                                   // signal that the CC1101 has received a
                                   // packet.
  WeatherDevice *current_device_;  // If receiving_from_tracked_device_ is set, 
                                   // indicates the device a packet is arriving
                                   // from.
  WeatherDevice *next_device_;  // When receive_timer_on_ is set, indicates
                                // the device the packet will be arriving from.

  byte search_channel_;  // The radio attempts to synchronize IDs on this
                         // channel. It is incremented if a CRC error occurs
                         // while synchronizing.
  bool debug_on_;  // Determines whether to print debugging information.
};

}  // namespace cc1101_weather_receiver

#endif  // CC1101_MODULE_H_

