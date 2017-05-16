#include <Arduino.h>
#include <Wire.h>
#include <Sodaq_UBlox_GPS.h>

#define MySerial        SERIAL_PORT_MONITOR
#define ARRAY_DIM(arr)  (sizeof(arr) / sizeof(arr[0]))

// List of interval values to be used in loop()
// to measure how long it takes to get a fix.
uint32_t intervals[] = {

        // Do a few tests with 1 minute delay
        1UL * 60 * 1000,
        1UL * 60 * 1000,
        1UL * 60 * 1000,

        // Try a few longer delays
        2UL * 60 * 1000,
        2UL * 60 * 1000,
        5UL * 60 * 1000,
        5UL * 60 * 1000,

        // Slowly increase the delays
        15UL * 60 * 1000,
        30UL * 60 * 1000,
        1UL * 60 * 60 * 1000,
        3UL * 60 * 60 * 1000,
        4UL * 60 * 60 * 1000,
        8UL * 60 * 60 * 1000,
};
size_t interval_ix = 0;

void find_fix(uint32_t delay_until);
void do_flash_led(int pin);

void setup()
{
    delay(3000);
    while (!SerialUSB) {
        // Wait for USB to connect
    }

    MySerial.begin(57600);

    digitalWrite(LED_RED, HIGH);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_GREEN, HIGH);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_BLUE, HIGH);
    pinMode(LED_BLUE, OUTPUT);

    do_flash_led(LED_RED);
    do_flash_led(LED_GREEN);
    do_flash_led(LED_BLUE);

    MySerial.println("SODAQ LoRaONE test_gps is starting ...");

    sodaq_gps.init(GPS_ENABLE);

    // This is for debugging to see more details, more messages
    // Use this in combination with setDiag()
    //sodaq_gps.setMinNumOfLines(10);

    // Uncomment the next line if you want to see the incoming $GPxxx messages
    //sodaq_gps.setDiag(MySerial);

    // First time finding a fix
    find_fix(0);
}

void loop()
{
    uint32_t wait_ms = intervals[interval_ix];
    if (++interval_ix > ARRAY_DIM(intervals)) {
        interval_ix = 0;
    }
    find_fix(wait_ms);
}

/*!
 * Find a GPS fix, but first wait a while
 */
void find_fix(uint32_t delay_until)
{
    MySerial.println(String("delay ... ") + delay_until + String("ms"));
    delay(delay_until);

    uint32_t start = millis();
    uint32_t timeout = 900L * 1000;
    MySerial.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
    if (sodaq_gps.scan(false, timeout)) {
        MySerial.println(String(" time to find fix: ") + (millis() - start) + String("ms"));
        MySerial.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
        MySerial.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
        MySerial.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
        MySerial.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));
    } else {
        MySerial.println("No Fix");
    }
}

void do_flash_led(int pin)
{
    for (size_t i = 0; i < 2; ++i) {
        delay(100);
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
}
