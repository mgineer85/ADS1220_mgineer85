/*
  Demonstrate different gains used for strain gauge application
  confirm that the result is almost equal if internalCalibration is issued after setting gain

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

    // setup strain gauge
    ads.setMux(ADS1220_MUX_AIN1_AIN2);
    ads.setVoltageReference(ADS1220_VREF_EXTERNAL_REFP1_REFN1);
    ads.setGain(ADS1220_GAIN_128);

    ads.internalCalibration();
}

void loop()
{
    int32_t reading;

    Serial.print("gain:128 ");
    ads.setGain(ADS1220_GAIN_128);
    ads.internalCalibration();
    reading = ads.getReading() / 128;
    Serial.println(reading);
    delay(50);

    Serial.print("gain:64  ");
    ads.setGain(ADS1220_GAIN_64);
    ads.internalCalibration();
    reading = ads.getReading() / 64;
    Serial.println(reading);
    delay(50);

    Serial.print("gain:32  ");
    ads.setGain(ADS1220_GAIN_32);
    ads.internalCalibration();
    reading = ads.getReading() / 32;
    Serial.println(reading);
    delay(50);

    Serial.print("gain:16  ");
    ads.setGain(ADS1220_GAIN_16);
    ads.internalCalibration();
    reading = ads.getReading() / 16;
    Serial.println(reading);
    delay(50);

    delay(1000);
}
