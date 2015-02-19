# CC1101-Weather-Receiver
![alt tag](ccwxrx_prototype.jpg)

The CC1101 Weather Receiver (CCWXRX) receives transmissions from Davis Instruments wireless weather devices.  It can receive data from up to eight Davis devices, the maximum number possible with their protocol.  The receiver is known to work with the Vantage Pro2 ISS and Leaf and Soil Moisture/Temperature Station, and should work with the Vantage Vue as well.  It transmits data to computer running weewx (weather software) for processing and archiving.  This module provides an inexpensive, open source solution for receiving data from Davis weather instruments.  It is based on work by Dekay (http://madscientistlabs.blogspot.com) who discovered and documented the protocol used by Davis and build a receiver using a Moteino (https://github.com/dekay/DavisRFM69).  This project uses a CC1101-CC1190 Evaluation Module from Texas Instruments to greatly improve range compared to the Moteino and allows transmissions from more than one Davis device to be tracked.  A BMP180 module (with the correct pinout) can be plugged into the PCB for pressure measurement.

The directory cc1101\_weather\_receiver contains code for an Arduino Pro Mini (3.3 V, 8 MHz) (ATmega328), which communicates with a linux node over serial.

A driver for weewx is provided in the ccwxrxvp2 directory.  Weewx is designed to run one instance for each weather station, but only a single program can read from a serial port.  Thus the ccwxrx_splitter, a program which reads data packets from the serial port and routes them to the correct weewx instance was created.

Design files for a PCB and case will be released soon.
