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

  _spiPort->begin();
  _SPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE1);

  pinMode(_CS, OUTPUT);
  pinMode(_DRDY, INPUT_PULLUP);
  digitalWrite(_CS, HIGH);

  // wait to ensure device is ready to communicate for sure.
  delay(50);

  bool result = true; // Accumulate a result as we do the setup

  result &= cmdReset();    // Reset all registers
  result &= isConnected(); // check connection
  // result &= command(ADS1220_START); // Start/Sync
  // result &= setConversionMode(ADS1220_CM_SINGLE_SHOT_MODE); // Power on analog and digital sections of the scale

  return result;
}

// Returns true if device is present
bool ADS1220::isConnected()
{
  // SPI cannot detect device presence - try to mod registers and check result
  // prerequisite: only on startup after cmdReset and registers are all 0
  // just a test if the ADS1220 is connected
  setBit(ADS1220_CTRL0_PGA_BYPASS, ADS1220_CTRL0);
  bool pgabypassed = getBit(ADS1220_CTRL0_PGA_BYPASS, ADS1220_CTRL0);
  clearBit(ADS1220_CTRL0_PGA_BYPASS, ADS1220_CTRL0);
  if (pgabypassed == true)
  {
    // Serial.println("OK, isConnected");
    return true;
  }
  else
  {
    // Serial.println("error not isConnected");
    return false;
  }
}

bool ADS1220::cmdStartSync()
{
  return command(ADS1220_START);
}
// Resets all registers to Power Of Defaults
bool ADS1220::cmdReset()
{

  bool retval = command(ADS1220_RESET);
  delay(1);

  // invalidate internal offset calibration
  setInternalCalibrationOffset(0);

  return retval;
}
bool ADS1220::cmdPowerDown()
{
  return command(ADS1220_PWRDOWN);
}

bool ADS1220::setPgaBypass(ADS1220_PGA_BYPASS_Values pga_bypass)
{
  setInternalCalibrationOffset(0);

  return setBit(ADS1220_CTRL0_PGA_BYPASS, ADS1220_CTRL0, pga_bypass);
}

bool ADS1220::setGain(ADS1220_GAIN_Values gain)
{
  // invalidate internal offset calibration because different for other gain setting
  setInternalCalibrationOffset(0);

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

// Returns true if Cycle Ready bit is set (conversion is complete)
bool ADS1220::waitUntilAvailable(uint16_t timeout_ms)
{
  uint32_t begin = millis();
  while (!available())
  {
    if (((millis() - begin) > 500)) // timeout 500ms
    {
      // Serial.println("abort send false");
      return false;
    }
  }

  // Serial.println("wait return true now");
  return true;
}

// Returns 24-bit reading
int32_t ADS1220::getReadingRaw()
{

  if (getBit(ADS1220_CTRL1_CM, ADS1220_CTRL1) == ADS1220_CM_SINGLE_SHOT_MODE)
  {
    cmdStartSync();
  }

  if (!waitUntilAvailable(500))
  {
    return INT_MAX;
  }

  _spiPort->beginTransaction(_SPISettings);
  digitalWrite(_CS, LOW);
  uint32_t valueRaw = (uint32_t)_spiPort->transfer(0x00) << 16; // MSB
  valueRaw |= (uint32_t)_spiPort->transfer(0x00) << 8;          // MidSB
  valueRaw |= (uint32_t)_spiPort->transfer(0x00);               // LSB
  digitalWrite(_CS, HIGH);
  _spiPort->endTransaction();

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
  return getReadingRaw() - getInternalCalibrationOffset();
}

int32_t ADS1220::getAverageReading(uint8_t number)
{
  return getAverageReadingRaw(number) - getInternalCalibrationOffset();
}

/// @brief Recommended to perform internal calibration after gain changed
/// @return calibration successful
bool ADS1220::internalCalibration()
{
  // helps improving accuracy by removing any offset from internal circuitry

  // remove pga offset by shorting mux and deduct offset afterwards from readings
  ADS1220_MUX_Values mux_before_calibration = getMux();
  // internally short mux - so read output shall be 0
  setMux(ADS1220_MUX_AINp_AINn_SHORTED_TO_AVDD_AVSS_DIV2);
  delay(10);
  // throw away to ensure stable results
  getAverageReadingRaw();
  // deviation from 0 is offset that is permanently removed from following readings
  setInternalCalibrationOffset(getAverageReadingRaw(8));
  // restore former mux config
  setMux(mux_before_calibration);

  return true;
}

void ADS1220::setInternalCalibrationOffset(int32_t val)
{
  _internal_calibration_offset = val;
}
int32_t ADS1220::getInternalCalibrationOffset()
{
  return _internal_calibration_offset;
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

void ADS1220::printRegisterValues()
{
  Serial.print("Config_Regs 0-3: ");
  Serial.print(readRegister(ADS1220_CTRL0), HEX);
  Serial.print(readRegister(ADS1220_CTRL1), HEX);
  Serial.print(readRegister(ADS1220_CTRL2), HEX);
  Serial.print(readRegister(ADS1220_CTRL3), HEX);
  Serial.println(" ");
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