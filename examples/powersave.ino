/*
  Basic Example

*/
#include <Arduino.h>
#include <Ads1220lib.h>

/* Create your ADS1220 object */
ADS1220 ads = ADS1220();

void setup()
{
    Serial.begin(9600);
    Serial.println("starting");

    if (!ads.begin())
    {
        Serial.println("Error connecting to device!");
    }
    else
    {
        Serial.println("Connected to device!");
    }

    // eco mode, turn off bridge excitation if no measurement
    ads.setPowerSwitch(ADS1220_PSW_AUTOMATIC);
    ads.setConversionMode(ADS1220_CM_SINGLE_SHOT_MODE);
}

void loop()
{
    int32_t reading = ads.getReading();
    ads.powerDown();
    Serial.println(reading);

    delay(1000);
}
