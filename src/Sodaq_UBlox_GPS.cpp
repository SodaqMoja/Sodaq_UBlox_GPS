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
const char Sodaq_UBlox_GPS::_fieldSep = ',';

static inline bool is_timedout(uint32_t from, uint32_t nr_ms) __attribute__((always_inline));
static inline bool is_timedout(uint32_t from, uint32_t nr_ms)
{
    return (millis() - from) > nr_ms;
}

Sodaq_UBlox_GPS::Sodaq_UBlox_GPS()
{
    _diagStream = 0;
    _addr = UBlox_I2C_addr;

    _seenLatLon = false;
    _numSatelites = 0;
    _lat = 0;
    _lon = 0;

    _seenTime = false;
    _hh = 0;
    _mm = 0;
    _ss = 0;

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
    _seenLatLon = false;
    _seenTime = false;
    _numSatelites = 0;
    _lat = 0;
    _lon = 0;

    while (!is_timedout(start, timeout) && (!_seenLatLon || !_seenTime)) {
        if (!readLine()) {
            // TODO Maybe quit?
            continue;
        }
        parseLine(_inputBuffer);
    }

    if (_seenTime) {
        debugPrintLn(String(" hh = ") + num2String(_hh, 2));
        debugPrintLn(String(" mm = ") + num2String(_mm, 2));
        debugPrintLn(String(" ss = ") + num2String(_ss, 2));
    }
    if (_seenLatLon) {
        debugPrintLn(String(" lat = ") + String(_lat, 7));
        debugPrintLn(String(" lon = ") + String(_lon, 7));
    }
    return retval;
}

bool Sodaq_UBlox_GPS::parseLine(const char * line)
{
    const char * typ;

    //debugPrintLn(String("= ") + line);
    if (!computeCrc(line, false)) {
        // Redo the check, with logging
        computeCrc(line, true);
        return false;
    }
    String data = line + 1;
    data.remove(data.length() - 3, 3);  // Strip checksum *<hex><hex>

    if (data.startsWith("GPGGA")) {
        return parseGPGGA(data);
    }

    if (data.startsWith("GPGSA")) {
        return parseGPGSA(data);
    }

    if (data.startsWith("GPRMC")) {
        return parseGPRMC(data);
    }

    if (data.startsWith("GPGSV")) {
        return parseGPGSV(data);
    }
    debugPrintLn(String("?? >> ") + line);
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
 * 0    $GPGGA
 * 1    time            hhmmss.ss       UTC time
 * 2    lat             ddmm.mmmmm      Latitude (degrees & minutes)
 * 3    NS              char            North/South indicator
 * 4    long            dddmm.mmmmm     Longitude (degrees & minutes)
 * 5    EW              char            East/West indicator
 * 6    quality         digit           Quality indicator for position fix: 0 No Fix, 6 Estimated, 1 Auto GNSS, 2 Diff GNSS
 * 7    numSV           num             Number of satellites used
 * 8    HDOP            num             Horizontal Dilution of Precision
 * 9    alt             num             Altitude above mean sea level
 * 10   uAlt            char            Altitude units: meters
 * 11   sep             num             Geoid separation: difference between geoid and mean sea level
 * 12   uSep            char            Separation units: meters
 * 13   diffAge         num             Age of differential corrections
 * 14   diffStation     num             ID of station providing differential corrections
 * 15   checksum        2 hex digits
 */
bool Sodaq_UBlox_GPS::parseGPGGA(const String & line)
{
    debugPrintLn("parseGPGGA");
    debugPrintLn(String(">> ") + line);
    if (getField(line, 6) != "0") {
        _lat = convertDegMinToDecDeg(getField(line, 2));
        if (getField(line, 3) == "S") {
            _lat = -_lat;
        }
        _lon = convertDegMinToDecDeg(getField(line, 4));
        if (getField(line, 5) == "W") {
            _lon = -_lon;
        }
        _seenLatLon = true;
    }

    String time = getField(line, 1);
    if (time.length() == 9) {
        _hh = time.substring(0, 2).toInt();
        _mm = time.substring(2, 4).toInt();
        _ss = time.substring(4, 6).toInt();
        _seenTime = true;
    }

    _numSatelites = getField(line, 7).toInt();
    return true;
}

/*!
 * Parse GPGSA line
 */
bool Sodaq_UBlox_GPS::parseGPGSA(const String & line)
{
    debugPrintLn("parseGPGSA");
    debugPrintLn(String(">> ") + line);
    return false;
}

/*!
 * Read the coordinates using $GPRMC
 * See also section 24.13 of u-blox 7, Receiver Description. Document number: GPS.G7-SW-12001-B
 *
 * 0    $GPRMC
 * 1    time            hhmmss.ss       UTC time
 * 2    status          char            Status, V = Navigation receiver warning, A = Data valid
 * 3    lat             ddmm.mmmmm      Latitude (degrees & minutes)
 * 4    NS              char            North/South
 * 5    long            dddmm.mmmmm     Longitude (degrees & minutes)
 * 6    EW              char            East/West
 * 7    spd             num             Speed over ground
 * 8    cog             num             Course over ground
 * 9    date            ddmmyy          Date in day, month, year format
 * 10   mv              num             Magnetic variation value
 * 11   mvEW            char            Magnetic variation E/W indicator
 * 12   posMode         char            Mode Indicator: 'N' No Fix, 'E' Estimate, 'A' Auto GNSS, 'D' Diff GNSS
 * 13   checksum        2 hex digits    Checksum
 */
bool Sodaq_UBlox_GPS::parseGPRMC(const String & line)
{
    debugPrintLn("parseGPRMC");
    debugPrintLn(String(">> ") + line);

    if (getField(line, 2) == "A" && getField(line, 12) != "N") {
        _lat = convertDegMinToDecDeg(getField(line, 3));
        if (getField(line, 4) == "S") {
            _lat = -_lat;
        }
        _lon = convertDegMinToDecDeg(getField(line, 5));
        if (getField(line, 4) == "W") {
            _lon = -_lon;
        }
        _seenLatLon = true;
    }

    String time = getField(line, 1);
    if (time.length() == 9) {
        _hh = time.substring(0, 2).toInt();
        _mm = time.substring(2, 4).toInt();
        _ss = time.substring(4, 6).toInt();
        _seenTime = true;
    }

    return false;
}

/*!
 * Parse GPGSV line
 */
bool Sodaq_UBlox_GPS::parseGPGSV(const String & line)
{
    debugPrintLn("parseGPGSV");
    debugPrintLn(String(">> ") + line);
    return false;
}

/*!
 * Compute and verify the checksum
 *
 * Each line must start with '$'
 * Each line must end with '*' <hex> <hex>
 */
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

String Sodaq_UBlox_GPS::num2String(int num, size_t width)
{
    String out;
    out = num;
    while (out.length() < width) {
        out = String("0") + out;
    }
    return out;
}

String Sodaq_UBlox_GPS::getField(const String & data, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == _fieldSep || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/*
 * Convert lat/long degree-minute format to decimal-degrees
 *
 * This code is from
 *   http://arduinodev.woofex.net/2013/02/06/adafruit_gps_forma/
 *
 * According to the NMEA Standard, Latitude and Longitude are output in the format Degrees, Minutes and
 * (Decimal) Fractions of Minutes. To convert to Degrees and Fractions of Degrees, or Degrees, Minutes, Seconds
 * and Fractions of seconds, the 'Minutes' and 'Fractional Minutes' parts need to be converted. In other words: If
 * the GPS Receiver reports a Latitude of 4717.112671 North and Longitude of 00833.914843 East, this is
 *   Latitude 47 Degrees, 17.112671 Minutes
 *   Longitude 8 Degrees, 33.914843 Minutes
 * or
 *   Latitude 47 Degrees, 17 Minutes, 6.76026 Seconds
 *   Longitude 8 Degrees, 33 Minutes, 54.89058 Seconds
 * or
 *   Latitude 47.28521118 Degrees
 *   Longitude 8.56524738 Degrees
 */
double Sodaq_UBlox_GPS::convertDegMinToDecDeg(const String & data)
{
    double degMin = data.toFloat();
    double min = 0.0;
    double decDeg = 0.0;

    //get the minutes, fmod() requires double
    min = fmod((double) degMin, 100.0);

    //rebuild coordinates in decimal degrees
    degMin = (int) (degMin / 100);
    decDeg = degMin + (min / 60);

    return decDeg;
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
