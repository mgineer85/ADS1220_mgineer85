/*
  Sensordetect example

  results should be equal for both, means that the burnout current does not affect the reading right after
  sensor detection test.

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

    ads.setConversionMode(ADS1220_CM_SINGLE_SHOT_MODE);

    // setup strain gauge
    ads.setMux(ADS1220_MUX_AIN1_AIN2);
    ads.setVoltageReference(ADS1220_VREF_EXTERNAL_REFP1_REFN1);
    ads.setGain(ADS1220_GAIN_128);
    ads.internalCalibration();
}

void loop()
{
    int32_t reading_with_previous_sensorcheck;
    int32_t reading_without_previous_sensorcheck;

    if (!ads.sensorConnected())
    {
        Serial.println("no sensor detected!");
    }
    reading_with_previous_sensorcheck = ads.getReading() / 128;
    delay(500);
    reading_without_previous_sensorcheck = ads.getReading() / 128;

    Serial.print("reading_with_previous_sensorcheck:    ");
    Serial.println(reading_with_previous_sensorcheck);
    Serial.print("reading_without_previous_sensorcheck: ");
    Serial.println(reading_without_previous_sensorcheck);

    delay(500);
}
