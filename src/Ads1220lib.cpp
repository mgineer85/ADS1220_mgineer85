/*

*/
#include <limits.h>
#include "Ads1220lib.h"

// Constructor
ADS1220::ADS1220() {}

// Sets up the ADS1220 for basic function
// Returns true upon completion
bool ADS1220::begin(uint8_t DRDY_pin, uint8_t CS_pin, SPIClass &spiPort)
{
  // Get user's options
  _DRDY = DRDY_pin;
  _CS = CS_pin;
  _spiPort = &spiPort;

  _spiPort->begin(_CS);
  _SPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE1);

  pinMode(_CS, OUTPUT);
  pinMode(_DRDY, INPUT_PULLUP);
  digitalWrite(_CS, HIGH);

  // if isnt connected, fail early.
  if (isConnected() == false) // Check for sensor
    return false;

  bool result = true; // Accumulate a result as we do the setup

  result &= reset();                                        // Reset all registers
  result &= setConversionMode(ADS1220_CM_SINGLE_SHOT_MODE); // Power on analog and digital sections of the scale

  return result;
}

// Returns true if device is present
bool ADS1220::isConnected()
{
  // SPI cannot detect device presence - try to get meas to check if connected

  if (getBit(ADS1220_CTRL1_CM, ADS1220_CTRL1) == ADS1220_CM_SINGLE_SHOT_MODE)
  {
    start_single_shot_measurement();
  }

  uint32_t begin = millis();

  while (!available())
  {
    if (((millis() - begin) > 500)) // timeout 500ms
    {
      return false;
    }
    delay(1);
  }

  return true;
}

bool ADS1220::start_single_shot_measurement()
{
  return command(ADS1220_START);
}
// Resets all registers to Power Of Defaults
bool ADS1220::reset()
{
  bool retval = command(ADS1220_RESET);
  delay(1);
  return retval;
}
bool ADS1220::powerDown()
{
  return command(ADS1220_PWRDOWN);
}

bool ADS1220::setPgaBypass(ADS1220_PGA_BYPASS_Values pga_bypass)
{
  return setBit(ADS1220_CTRL0_PGA_BYPASS, ADS1220_CTRL0, pga_bypass);
}
bool ADS1220::setGain(ADS1220_GAIN_Values gain)
{
  uint8_t value = readRegister(ADS1220_CTRL0);
  value &= 0b11110001;                 // Clear gain bits
  value |= gain << ADS1220_CTRL0_GAIN; // Mask in new bits
  return (writeRegister(ADS1220_CTRL0, value));
}
bool ADS1220::setMux(ADS1220_MUX_Values mux)
{
  uint8_t value = readRegister(ADS1220_CTRL0);
  value &= 0b00001111;               // Clear gain bits
  value |= mux << ADS1220_CTRL0_MUX; // Mask in new bits
  return (writeRegister(ADS1220_CTRL0, value));
}

ADS1220_MUX_Values ADS1220::getMux()
{
  uint8_t value = readRegister(ADS1220_CTRL0);
  value &= 0b11110000;                // Clear gain bits
  value = value >> ADS1220_CTRL0_MUX; // Mask in new bits
  return (ADS1220_MUX_Values)value;
}

bool ADS1220::setBurnoutCurrentSource(ADS1220_BCS_Values bo)
{
  return setBit(ADS1220_CTRL1_BCS, ADS1220_CTRL1, bo);
}
bool ADS1220::setTemperatureSensorMode(ADS1220_TS_Values ts)
{
  return setBit(ADS1220_CTRL1_TS, ADS1220_CTRL1, ts);
}
bool ADS1220::setConversionMode(ADS1220_CM_Values cm)
{
  return setBit(ADS1220_CTRL1_CM, ADS1220_CTRL1, cm);
}
bool ADS1220::setOperatingMode(ADS1220_MODE_Values mode)
{
  uint8_t value = readRegister(ADS1220_CTRL1);
  value &= 0b11100111;                 // Clear gain bits
  value |= mode << ADS1220_CTRL1_MODE; // Mask in new bits
  return (writeRegister(ADS1220_CTRL1, value));
}
bool ADS1220::setDataRate(ADS1220_DR_Values dr)
{
  uint8_t value = readRegister(ADS1220_CTRL1);
  value &= 0b00011111;             // Clear gain bits
  value |= dr << ADS1220_CTRL1_DR; // Mask in new bits
  return (writeRegister(ADS1220_CTRL1, value));
}

bool ADS1220::setIdacCurrent(ADS1220_IDAC_Values idac)
{
  uint8_t value = readRegister(ADS1220_CTRL2);
  value &= 0b11111000;                 // Clear gain bits
  value |= idac << ADS1220_CTRL2_IDAC; // Mask in new bits
  return (writeRegister(ADS1220_CTRL2, value));
}
bool ADS1220::setPowerSwitch(ADS1220_PSW_Values psw)
{
  return setBit(ADS1220_CTRL2_PSW, ADS1220_CTRL2, psw);
}
bool ADS1220::setFirFilter(ADS1220_FIR_50_60_Values fir_mode)
{
  uint8_t value = readRegister(ADS1220_CTRL2);
  value &= 0b11001111;                          // Clear gain bits
  value |= fir_mode << ADS1220_CTRL2_FIR_50_60; // Mask in new bits
  return (writeRegister(ADS1220_CTRL2, value));
}
bool ADS1220::setVoltageReference(ADS1220_VREF_Values vref)
{
  uint8_t value = readRegister(ADS1220_CTRL2);
  value &= 0b00111111;                 // Clear gain bits
  value |= vref << ADS1220_CTRL2_VREF; // Mask in new bits
  return (writeRegister(ADS1220_CTRL2, value));
}

bool ADS1220::setDataReadyMode(ADS1220_DRDYM_Values drdym)
{
  return setBit(ADS1220_CTRL3_DRDYM, ADS1220_CTRL3, drdym);
}
bool ADS1220::setIDAC2Routing(ADS1220_I2MUX_Values i2mux)
{
  uint8_t value = readRegister(ADS1220_CTRL3);
  value &= 0b11100011;                   // Clear gain bits
  value |= i2mux << ADS1220_CTRL3_I1MUX; // Mask in new bits
  return (writeRegister(ADS1220_CTRL3, value));
}
bool ADS1220::setIDAC1Routing(ADS1220_I1MUX_Values i1mux)
{
  uint8_t value = readRegister(ADS1220_CTRL3);
  value &= 0b00011111;                   // Clear gain bits
  value |= i1mux << ADS1220_CTRL3_I2MUX; // Mask in new bits
  return (writeRegister(ADS1220_CTRL3, value));
}

// Returns true if Cycle Ready bit is set (conversion is complete)
bool ADS1220::available()
{
  return digitalRead(_DRDY) == LOW;
}

// Returns 24-bit reading
int32_t ADS1220::getReadingRaw()
{

  if (getBit(ADS1220_CTRL1_CM, ADS1220_CTRL1) == ADS1220_CM_SINGLE_SHOT_MODE)
  {
    start_single_shot_measurement();
  }

  uint32_t begin = millis();
  while (!available())
  {
    if (((millis() - begin) > 500)) // timeout 500ms
    {
      Serial.println("abort send INT_MAX");
      return INT_MAX;
    }
  }

  _spiPort->beginTransaction(_SPISettings);
  digitalWrite(_CS, LOW);
  uint32_t valueRaw = (uint32_t)_spiPort->transfer(0x00) << 16; // MSB
  valueRaw |= (uint32_t)_spiPort->transfer(0x00) << 8;          // MidSB
  valueRaw |= (uint32_t)_spiPort->transfer(0x00);               // LSB
  digitalWrite(_CS, HIGH);
  _spiPort->endTransaction();

  // TODO: check: could be -1 if 0 is actually output!
  int32_t valueShifted = (int32_t)(valueRaw << 8);
  int32_t value = (valueShifted >> 8);

  return (value);
}

int32_t ADS1220::getAverageReadingRaw(uint8_t number)
{

  int64_t total = 0; // warning: can overflow in theory!

  for (int i = 0; i < number; i++)
  {
    total += getReadingRaw();
  }

  return total / number;
}

int32_t ADS1220::getReading()
{
  return getReadingRaw() - _internal_calibration_offset;
}

bool ADS1220::internalCalibration()
{
  // helps improving accuracy by removing any offset from internal circuitry

  // 1: remove pga offset by shorting mux and deduct offset afterwards from readings
  ADS1220_MUX_Values mux_before_calibration = getMux();

  setMux(ADS1220_MUX_AINp_AINn_SHORTED_TO_AVDD_AVSS_DIV2);
  getAverageReadingRaw(2); // throw away to ensure stable results
  _internal_calibration_offset = getAverageReadingRaw();

  setMux(mux_before_calibration); // restore former mux config

  // Serial.print("offset cali: ");
  // Serial.println(_internal_calibration_offset);

  return true;
}

bool ADS1220::sensorConnected()
{
  setBurnoutCurrentSource(ADS1220_BCS_CURRENT_SOURCE_ON);
  int32_t reading = getReadingRaw();
  setBurnoutCurrentSource(ADS1220_BCS_CURRENT_SOURCE_OFF);
  if (reading >= 0x7FFFFF)
    return false;
  else
    return true;
}

/* *********************************************************
 *  SPI communication methods
 ********************************************************* */

// Mask & set a given bit within a register
bool ADS1220::setBit(uint8_t bitNumber, uint8_t registerAddress, bool value)
{
  if (value)
  {
    return setBit(bitNumber, registerAddress);
  }
  else
  {
    return clearBit(bitNumber, registerAddress);
  }
}

// Mask & set a given bit within a register
bool ADS1220::setBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = readRegister(registerAddress);
  value |= (1 << bitNumber); // Set this bit
  return (writeRegister(registerAddress, value));
}

// Mask & clear a given bit within a register
bool ADS1220::clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = readRegister(registerAddress);
  value &= ~(1 << bitNumber); // Set this bit
  return (writeRegister(registerAddress, value));
}

// Return a given bit within a register
bool ADS1220::getBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = readRegister(registerAddress);
  value &= (1 << bitNumber); // Clear all but this bit
  return (value);
}

uint8_t ADS1220::readRegister(uint8_t reg)
{
  uint8_t regValue;

  _spiPort->beginTransaction(_SPISettings);
  digitalWrite(_CS, LOW);
  _spiPort->transfer(ADS1220_RREG | (reg << 2));
  regValue = _spiPort->transfer(0x00);
  digitalWrite(_CS, HIGH);
  _spiPort->endTransaction();

  return regValue;
}

bool ADS1220::writeRegister(uint8_t reg, uint8_t val)
{
  _spiPort->beginTransaction(_SPISettings);
  digitalWrite(_CS, LOW);
  _spiPort->transfer(ADS1220_WREG | (reg << 2));
  _spiPort->transfer(val);
  digitalWrite(_CS, HIGH);
  _spiPort->endTransaction();

  return true; // always return true because SPI errors cannot be detected
}

bool ADS1220::command(uint8_t cmd)
{
  _spiPort->beginTransaction(_SPISettings);
  digitalWrite(_CS, LOW);
  _spiPort->transfer(cmd);
  digitalWrite(_CS, HIGH);
  _spiPort->endTransaction();

  return true; // always return true because SPI errors cannot be detected
}