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

#include <Arduino.h>
#include <Wire.h>
#include "Sodaq_UBlox_GPS.h"

#define DEBUG 1
#ifdef DEBUG
#define debugPrintLn(...) { if (this->_diagStream) this->_diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (this->_diagStream) this->_diagStream->print(__VA_ARGS__); }
#else
#define debugPrintLn(...)
#define debugPrint(...)
#endif

static const size_t INPUT_BUFFER_SIZE = 128;    // TODO Check UBlox manual ReceiverDescProtSpec
static const uint8_t UBlox_I2C_addr = 0x42;

#define GPS_ENABLE_ON   HIGH
#define GPS_ENABLE_OFF  LOW

Sodaq_UBlox_GPS sodaq_gps;

static inline bool is_timedout(uint32_t from, uint32_t nr_ms) __attribute__((always_inline));
static inline bool is_timedout(uint32_t from, uint32_t nr_ms)
{
    return (millis() - from) > nr_ms;
}

Sodaq_UBlox_GPS::Sodaq_UBlox_GPS()
{
    _diagStream = 0;
    _addr = UBlox_I2C_addr;
    _gotAllMessages = false;
    _numSatelites = 0;
    _trans_active = false;

    _inputBuffer = static_cast<char*>(malloc(INPUT_BUFFER_SIZE));
    _inputBufferSize = INPUT_BUFFER_SIZE;
}

void Sodaq_UBlox_GPS::init()
{
    Wire.begin();
    digitalWrite(GPS_ENABLE, GPS_ENABLE_OFF);
    pinMode(GPS_ENABLE, OUTPUT);

    on();
    delay(500);         // TODO Is this needed?
}

/*!
 *
 */
bool Sodaq_UBlox_GPS::scan(uint32_t timeout)
{
    bool retval = false;
    uint32_t start = millis();
    _gotAllMessages = false;
    _numSatelites = 0;
    while (!is_timedout(start, timeout) && !_gotAllMessages) {
        if (!readLine()) {
            // TODO Maybe quit?
            continue;
        }
        debugPrintLn(String(">> ") + _inputBuffer);
        parseLine(_inputBuffer);
    }
    return retval;
}

bool Sodaq_UBlox_GPS::parseLine(const char * line)
{
    const char * typ;

    if (!computeCrc(line, false)) {
        // Redo the check, with logging
        computeCrc(line, true);
        return false;
    }
    typ = "$GPGGA";
    if (strncmp(line, typ, strlen(typ)) == 0) {
        return parseGPGGA(line);
    }
    typ = "$GPGSA";
    if (strncmp(line, typ, strlen(typ)) == 0) {
        return parseGPGSA(line);
    }
    typ = "$GPRMC";
    if (strncmp(line, typ, strlen(typ)) == 0) {
        return parseGPRMC(line);
    }
    return false;
}

/*!
 * Read the coordinates using $GPGGA
 * See also section 24.3 of u-blox 7, Receiver Description. Document number: GPS.G7-SW-12001-B
 *
 * See section "Decode of selected position sentences" at
 *   http://www.gpsinformation.org/dale/nmea.htm
 * The most important NMEA sentences include the GGA which provides the
 * current Fix data, the RMC which provides the minimum gps sentences
 * information, and the GSA which provides the Satellite status data.
 *
 * GGA - essential fix data which provide 3D location and accuracy data.
 *  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 * Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
     *47          the checksum data, always begins with *

 * If the height of geoid is missing then the altitude should be suspect. Some
 * non-standard implementations report altitude with respect to the ellipsoid
 * rather than geoid altitude. Some units do not report negative altitudes at
 * all. This is the only sentence that reports altitude.
 */
bool Sodaq_UBlox_GPS::parseGPGGA(const String & line)
{
    debugPrintLn("parseGPGGA");
    for (size_t fld = 0; fld < 11; ++fld) {
        debugPrintLn(String("fld ") + fld + " \"" + getField(line, ',', fld) + "\"");
    }

    // Time HHMMSS
    (uint32_t)getField(line, ',', 1).toFloat();

    // Check GPS Quality
    if (getField(line, ',', 6) != "0") {
        // Not worth looking any further ??
        // return false;
    }
    _numSatelites = getField(line, ',', 7).toInt();
    return true;
}

/*!
 * Parse GPGSA line
 */
bool Sodaq_UBlox_GPS::parseGPGSA(const char * line)
{
    debugPrintLn("parseGPGSA");
    return false;
}

/*!
 * Read the coordinates using $GPRMC
 * See also section 24.13 of u-blox 7, Receiver Description. Document number: GPS.G7-SW-12001-B
 *
 * 0    $GPRMC
 * 1    time            hhmmss.ss       UTC time
 * 2    status          char            Status
 * 3    lat             ddmm.mmmmm      Latitude (degrees & minutes)
 * 4    NS              char            North/South
 * 5    long            dddmm.mmmmm     Longitude (degrees & minutes)
 * 6    EW              char            East/West
 * 7    spd             num             Speed over ground
 * 8    cog             num             Course over ground
 * 9    date            ddmmyy          Date in day, month, year format
 * 10   mv              num             Magnetic variation value
 * 11   mvEW            char            Magnetic variation E/W indicator
 * 12   posMode         char            Mode Indicator
 * 13   checksum
 */
bool Sodaq_UBlox_GPS::parseGPRMC(const char * line)
{
    debugPrintLn("parseGPRMC");
    return false;
}

bool Sodaq_UBlox_GPS::computeCrc(const char * line, bool do_logging)
{
    if (do_logging) {
        debugPrint(line);
    }
    size_t len = strlen(line);
    if (len < 4) {
        if (do_logging) {
            debugPrint("  Invalid short: ");
            debugPrintLn(len);
        }
        return false;
    }
    if (line[0] != '$') {
        if (do_logging) {
            debugPrintLn("  Invalid$");
        }
        return false;
    }
    if (line[len - 3] != '*') {
        if (do_logging) {
            debugPrintLn("  Invalid*");
        }
        return false;
    }

    uint8_t crc = 0;
    for (size_t i = 1; i < len - 3; ++i) {
        crc ^= line[i];
    }

    uint8_t crc1 = getHex2(line, len - 2);
    if (crc != crc1) {
        if (do_logging) {
            debugPrint("  INVALID CRC ");
            debugPrint(crc1, HEX);
            debugPrint("  EXPECTED ");
            debugPrintLn(crc, HEX);
        }
        return false;
    }

    //debugSerial.println("  OK");
    return true;
}

uint8_t Sodaq_UBlox_GPS::getHex2(const char * s, size_t index)
{
    uint8_t val = 0;
    char c;
    c = s[index];
    if (c >= '0' && c <= '9') {
        val += c - '0';
    } else if (c >= 'a' && c <= 'f') {
        val += c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        val += c - 'A' + 10;
    }
    val <<= 4;
    c = s[++index];
    if (c >= '0' && c <= '9') {
        val += c - '0';
    } else if (c >= 'a' && c <= 'f') {
        val += c - 'a' + 10;
    } else if (c >= 'A' && c <= 'F') {
        val += c - 'A' + 10;
    }
    return val;
}

String Sodaq_UBlox_GPS::getField(const String & data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/*!
 * Read one NMEA frame
 */
bool Sodaq_UBlox_GPS::readLine(uint32_t timeout)
{
    if (!_inputBuffer) {
        return false;
    }

    uint32_t start = millis();
    char c;
    char *ptr = _inputBuffer;
    size_t cnt = 0;
    *ptr = '\0';

    c = 0;
    while (!is_timedout(start, timeout)) {
        c = (char)read();
        if (c == '$') {
            break;
        }
    }
    if (c != '$') {
        return false;
    }
    *ptr++ = c;
    ++cnt;

    c = 0;
    while (!is_timedout(start, timeout)) {
        c = (char)read();
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            break;
        }
        if (cnt < _inputBufferSize - 1) {
            *ptr++ = c;
            ++cnt;
        }
    }
    *ptr = '\0';
    if (c != '\n') {
        return false;
    }
    endTransmission();
    return true;
}

uint8_t Sodaq_UBlox_GPS::read()
{
    beginTransmission();

    uint8_t b = 0xFF;
    uint8_t nr_bytes;
    nr_bytes = Wire.requestFrom(_addr, 1, false);
    if (nr_bytes == 1) {
        b = Wire.read();
    }
    return b;
}

void Sodaq_UBlox_GPS::beginTransmission()
{
    if (_trans_active) {
        return;
    }
    Wire.beginTransmission(_addr);
    _trans_active = true;
}

void Sodaq_UBlox_GPS::endTransmission()
{
    if (!_trans_active) {
        return;
    }
    Wire.endTransmission();
    _trans_active = false;
}

void Sodaq_UBlox_GPS::on()
{
    digitalWrite(GPS_ENABLE, GPS_ENABLE_ON);
}

void Sodaq_UBlox_GPS::off()
{
    digitalWrite(GPS_ENABLE, GPS_ENABLE_OFF);
}
