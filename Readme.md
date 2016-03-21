# Sodaq_ublox_gps

Arduino library for using the UBlox xxx.

## Usage

Quick example:

```c
#include "Sodaq_UBlox_GPS.h"

```

Method|Description
------|------
**getDefaultBaudRate ()**|Returns the correct baudrate for the serial port that connects to the device.
**setDiag (Stream& stream)**|Sets the optional "Diagnostics and Debug" stream.
**initOTA (Stream& stream, const uint8_t devEUI[8], const uint8_t appEUI[8], const uint8_t appKey[16], bool adr = true)**|Initializes the device and connects to the network using Over-The-Air Activation. Returns true on successful connection.
**initABP (Stream& stream, const uint8_t devAddr[4], const uint8_t appSKey[16], const uint8_t nwkSKey[16], bool adr = true)**|Initializes the device and connects to the network using Activation By Personalization. Returns true on successful connection.
**send (uint8_t port, const uint8_t\* payload, uint8_t size)**|Sends the given payload without acknowledgement. Returns 0 (NoError) when transmission is successful or one of the MacTransmitErrorCodes otherwise.
**sendReqAck (uint8_t port, const uint8_t\* payload, uint8_t size, uint8_t maxRetries)**|Sends the given payload with acknowledgement. Returns 0 (NoError) when transmission is successful or one of the MacTransmitErrorCodes otherwise.
**receive (uint8_t\* buffer, uint16_t size, uint16_t payloadStartPosition = 0)**|Copies the latest received packet (optionally starting from the "payloadStartPosition" position of the payload) into the given "buffer", up to "size" number of bytes. Returns the number of bytes written or 0 if no packet is received since last transmission.

###### And only if using dynamic buffers (USE_DYNAMIC_BUFFERS is defined in the library - not by default)
Method|Description
------|------
**setInputBufferSize (uint16_t value)**|Sets the size of the input buffer. Needs to be called before initOTA()/initABP().
**setReceivedPayloadBufferSize (uint16_t value)**|Sets the size of the "Received Payload" buffer. Needs to be called before initOTA()/initABP().


## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request

## History

v1.0.0 Initial Release

## License

Copyright (c) 2016 SODAQ. All rights reserved.

This file is part of Sodaq_UBlox_GPS.

Sodaq_RN2483 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or(at your option) any later version.

Sodaq_RN2483 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with Sodaq_RN2483.  If not, see
<http://www.gnu.org/licenses/>.
