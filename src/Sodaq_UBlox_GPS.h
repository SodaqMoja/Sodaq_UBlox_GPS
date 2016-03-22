/*
* Copyright (c) 2016 SODAQ. All rights reserved.
*
* This file is part of Sodaq_UBlox_GPS.
*
* Sodaq_UBlox_GPS is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published
* by the Free Software Foundation, either version 3 of the License, or (at
* your option) any later version.
*
* Sodaq_UBlox_GPS is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
* License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Sodaq_UBlox_GPS.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _Sodaq_RN2483_h
#define _Sodaq_RN2483_h

#include <WString.h>
#include <stdint.h>

class Sodaq_UBlox_GPS
{
public:
    Sodaq_UBlox_GPS();

    void init();
    bool scan(uint32_t timeout=20000);

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream &stream) { _diagStream = &stream; }
    void setDiag(Stream *stream) { _diagStream = stream; }

private:
    void on();
    void off();

    // Read one byte
    uint8_t read();
    bool readLine(uint32_t timeout = 10000);
    bool parseLine(const char * line);
    bool parseGPGGA(const String & line);
    bool parseGPGSA(const String & line);
    bool parseGPRMC(const String & line);
    bool parseGPGSV(const String & line);
    bool computeCrc(const char * line, bool do_logging=false);
    uint8_t getHex2(const char * s, size_t index);
    String getField(const String & data, int index);
    String getNumField(const String & data, int index);
    double convertDegMinToDecDeg(const String & data);
    String num2String(int num, size_t width);

    void beginTransmission();
    void endTransmission();

    // The (optional) stream to show debug information.
    Stream *    _diagStream;

    uint8_t     _addr;

    bool        _seenLatLon;
    uint8_t     _numSatelites;
    double      _lat;
    double      _lon;

    bool        _seenTime;
    uint8_t     _hh;
    uint8_t     _mm;
    uint8_t     _ss;

    bool        _trans_active;

    static const char _fieldSep;
    char *      _inputBuffer;
    size_t      _inputBufferSize;
};

extern Sodaq_UBlox_GPS sodaq_gps;

#endif // Sodaq_RN2483
