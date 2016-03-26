#include <Arduino.h>
#include <Sodaq_UBlox_GPS.h>

#define MySerial        SERIAL_PORT_MONITOR
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

    sodaq_gps.init();
    sodaq_gps.setDiag(MySerial);
}

void loop()
{
    uint32_t start = millis();
    MySerial.println("waiting for fix ...");
    if (sodaq_gps.scan(false, 120000)) {
        MySerial.println(String(" fix took: ") + (millis() - start) + String("ms"));
        MySerial.println(String(" hhmmss = ") + sodaq_gps.getTimeString());
        MySerial.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
        MySerial.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
    } else {
        MySerial.println("No Fix");
    }
    uint32_t wait_ms = 60000;
    MySerial.println(String("delay ...") + wait_ms + String("ms"));
    delay(wait_ms);
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
