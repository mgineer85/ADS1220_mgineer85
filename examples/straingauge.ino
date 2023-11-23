/*
  Strain Gauge Optimized Example

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

    // setup strain gauge
    ads.setMux(ADS1220_MUX_AIN1_AIN2);
    ads.setVoltageReference(ADS1220_VREF_EXTERNAL_REFP1_REFN1);
    ads.setGain(ADS1220_GAIN_128);
    ads.internalCalibration();
}

void loop()
{
    int32_t reading;

    if (!ads.sensorConnected())
    {
        Serial.println("no sensor detected!");
    }
    else
    {
        delay(500);
        reading = ads.getReading() / 128;
        ads.powerDown();
        Serial.print("reading: ");
        Serial.println(reading);
    }

    delay(1000);
}
