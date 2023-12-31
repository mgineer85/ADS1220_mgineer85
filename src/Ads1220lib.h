/*

*/

#pragma once

#include <Arduino.h>
#include <SPI.h>

#define ADS1220_CS_PIN_DEFAULT 8    // 7   // chip select pin
#define ADS1220_DRDY_PIN_DEFAULT 14 // 6 // data ready pin

// ADS1220 SPI commands
typedef enum
{
  ADS1220_RESET = 0b00000110,   // Reset the device
  ADS1220_START = 0b00001000,   // Start or restart conversions
  ADS1220_PWRDOWN = 0b00000010, // Enter power-down mode
  ADS1220_RDATA = 0b00010000,   // Read data by command
  ADS1220_RREG = 0b00100000,    // Read nn registers starting at address rr
  ADS1220_WREG = 0b01000000,    // Write nn registers starting at address rr
} Ads1220_Commands;

// Register Map
typedef enum
{
  ADS1220_CTRL0 = 0x00,
  ADS1220_CTRL1,
  ADS1220_CTRL2,
  ADS1220_CTRL3,
} Ads1220_Registers;

// Bits within the CTRL0 register
typedef enum
{
  ADS1220_CTRL0_PGA_BYPASS = 0, // [0]
  ADS1220_CTRL0_GAIN = 1,       // [3:1]
  ADS1220_CTRL0_MUX = 4,        // [7:4]
} CTRL0_Bits;

// Bits within the CTRL1 register
typedef enum
{
  ADS1220_CTRL1_BCS = 0,  // [0]
  ADS1220_CTRL1_TS = 1,   // [1]
  ADS1220_CTRL1_CM = 2,   // [2]
  ADS1220_CTRL1_MODE = 3, // [4:3]
  ADS1220_CTRL1_DR = 5,   // [7:5]
} CTRL1_Bits;

// Bits within the CTRL2 register
typedef enum
{
  ADS1220_CTRL2_IDAC = 0,      // [2:0]
  ADS1220_CTRL2_PSW = 3,       // [3]
  ADS1220_CTRL2_FIR_50_60 = 4, // [5:4]
  ADS1220_CTRL2_VREF = 6,      // [7:6]
} CTRL2_Bits;

// Bits within the CTRL3 register
typedef enum
{
  ADS1220_CTRL3_RESERVED = 0, // [0]
  ADS1220_CTRL3_DRDYM = 1,    // [1]
  ADS1220_CTRL3_I2MUX = 2,    // [4:2]
  ADS1220_CTRL3_I1MUX = 5,    // [7:5]
} CTRL3_Bits;

// CTRL0[0] Disables and bypasses the internal low-noise PGA
typedef enum
{
  ADS1220_PGA_ENABLED = 0, // default
  ADS1220_PGA_DISABLED_BYPASSED = 1,
} ADS1220_PGA_BYPASS_Values;

// CTRL0[3:1] Gain configuration
typedef enum
{
  ADS1220_GAIN_1 = 0b000, // default
  ADS1220_GAIN_2 = 0b001,
  ADS1220_GAIN_4 = 0b010,
  ADS1220_GAIN_8 = 0b011,
  ADS1220_GAIN_16 = 0b100,
  ADS1220_GAIN_32 = 0b101,
  ADS1220_GAIN_64 = 0b110,
  ADS1220_GAIN_128 = 0b111,
} ADS1220_GAIN_Values;

// CTRL0[7:4] Input multiplexer configuration
typedef enum
{
  ADS1220_MUX_AIN0_AIN1 = 0b0000, // default
  ADS1220_MUX_AIN0_AIN2,
  ADS1220_MUX_AIN0_AIN3,
  ADS1220_MUX_AIN1_AIN2,
  ADS1220_MUX_AIN1_AIN3,
  ADS1220_MUX_AIN2_AIN3,
  ADS1220_MUX_AIN1_AIN0,
  ADS1220_MUX_AIN3_AIN2,
  ADS1220_MUX_AIN0_AVSS,
  ADS1220_MUX_AIN1_AVSS,
  ADS1220_MUX_AIN2_AVSS,
  ADS1220_MUX_AIN3_AVSS,
  ADS1220_MUX_VREFPx_VREFNx_DIV4_MONITOR_PGA_BYPASSED,
  ADS1220_MUX_AVDD_AVSS_DIV4_MONITOR_PGA_BYPASSED,
  ADS1220_MUX_AINp_AINn_SHORTED_TO_AVDD_AVSS_DIV2,
  // ADS1220_MUX_RESERVED,
} ADS1220_MUX_Values;

// CTRL1[0] Burn-out current sources
typedef enum
{
  ADS1220_BCS_CURRENT_SOURCE_OFF = 0, // default
  ADS1220_BCS_CURRENT_SOURCE_ON = 1,
} ADS1220_BCS_Values;

// CTRL1[1] Temperature sensor mode
typedef enum
{
  ADS1220_TS_TEMP_SENSOR_DISABLED = 0, // default
  ADS1220_TS_TEMP_SENSOR_ENABLED = 1,
} ADS1220_TS_Values;

// CTRL1[2] Conversion mode
typedef enum
{
  ADS1220_CM_SINGLE_SHOT_MODE = 0, // default
  ADS1220_CM_CONTINUOUS_CONVERSION_MODE = 1,
} ADS1220_CM_Values;

// CTRL1[4:3] Operating mode
typedef enum
{
  ADS1220_MODE_NORMAL = 0b00, // default
  ADS1220_MODE_DUTY_CYCLE = 0b01,
  ADS1220_MODE_TURBO = 0b10,
  // ADS1220_MODE_RESERVED = 0b11,
} ADS1220_MODE_Values;

// CTRL1[7:5] Data rate depends on operating mode
typedef enum
{
  ADS1220_DR_NORMAL20_DUTY5_TURBO40 = 0b000, // default
  ADS1220_DR_NORMAL45_DUTY11_TURBO90 = 0b001,
  ADS1220_DR_NORMAL90_DUTY22_TURBO180 = 0b010,
  ADS1220_DR_NORMAL175_DUTY44_TURBO350 = 0b011,
  ADS1220_DR_NORMAL330_DUTY82_TURBO660 = 0b100,
  ADS1220_DR_NORMAL600_DUTY150_TURBO1200 = 0b101,
  ADS1220_DR_NORMAL1000_DUTY250_TURBO2000 = 0b110,
  // ADS1220_DR_RESERVED = 0b111,
} ADS1220_DR_Values;

// CTRL2[2:0] IDAC current setting
typedef enum
{
  ADS1220_IDAC_OFF = 0b000, // default
  ADS1220_IDAC_10uA = 0b001,
  ADS1220_IDAC_50uA = 0b010,
  ADS1220_IDAC_100uA = 0b011,
  ADS1220_IDAC_250uA = 0b100,
  ADS1220_IDAC_500uA = 0b101,
  ADS1220_IDAC_1000uA = 0b110,
  ADS1220_IDAC_1500uA = 0b111,
} ADS1220_IDAC_Values;

// CTRL2[3] Low-side power switch configuration
typedef enum
{
  ADS1220_PSW_ALWAYS_OPEN = 0, // default
  ADS1220_PSW_AUTOMATIC = 1,
} ADS1220_PSW_Values;

// CTRL2[5:4] FIR filter configuration
typedef enum
{
  ADS1220_FIR_50_60_REJECTION_OFF = 0b00, // default
  ADS1220_FIR_50_60_REJECT_SIMULTANEOUS = 0b01,
  ADS1220_FIR_REJECT_50_ONLY = 0b10,
  ADS1220_FIR_REJECT_60_ONLY = 0b11,
} ADS1220_FIR_50_60_Values;

// CTRL2[7:6] Voltage reference selection
typedef enum
{
  ADS1220_VREF_INTERNAL_2048 = 0b00, // default
  ADS1220_VREF_EXTERNAL_REFP0_REFN0 = 0b01,
  ADS1220_VREF_EXTERNAL_REFP1_REFN1 = 0b10,
  ADS1220_VREF_ANALOG_SUPPY_AVDD_AVSS = 0b11,
} ADS1220_VREF_Values;

// CTRL3[0] Reserved

// CTRL3[1] DRDY mode
typedef enum
{
  ADS1220_DRDYM_PIN_DRDY_ONLY = 0, // default
  ADS1220_DRDYM_PIN_DRDY_AND_DOUT = 1,
} ADS1220_DRDYM_Values;

// CTRL3[4:2] IDAC2 routing configuration
typedef enum
{
  ADS1220_I2MUX_DISABLED = 0b000, // default
  ADS1220_I2MUX_AIN0_REFP1 = 0b001,
  ADS1220_I2MUX_AIN1 = 0b010,
  ADS1220_I2MUX_AIN2 = 0b011,
  ADS1220_I2MUX_AIN3_REFN1 = 0b100,
  ADS1220_I2MUX_REFP0 = 0b101,
  ADS1220_I2MUX_REFN0 = 0b110,
  // ADS1220_I2MUX_RESERVED = 0b111,
} ADS1220_I2MUX_Values;

// CTRL3[7:5] IDAC1 routing configuration
typedef enum
{
  ADS1220_I1MUX_DISABLED = 0b000, // default
  ADS1220_I1MUX_AIN0_REFP1 = 0b001,
  ADS1220_I1MUX_AIN1 = 0b010,
  ADS1220_I1MUX_AIN2 = 0b011,
  ADS1220_I1MUX_AIN3_REFN1 = 0b100,
  ADS1220_I1MUX_REFP0 = 0b101,
  ADS1220_I1MUX_REFN0 = 0b110,
  // ADS1220_I2MUX_RESERVED = 0b111,
} ADS1220_I1MUX_Values;

class ADS1220
{
  /* *************************************************** PUBLIC *************************************************** */

public: // basic functions and communication related
  ADS1220();

  bool begin(uint8_t DRDY_pin = ADS1220_DRDY_PIN_DEFAULT, uint8_t CS_pin = ADS1220_CS_PIN_DEFAULT, SPIClass &spiPort = SPI);
  bool isConnected(); // Returns true if device is detected

public: // ADS1220 related functions
  // configure register functions
  bool setPgaBypass(ADS1220_PGA_BYPASS_Values pga_bypass); // CONF0:
  bool setGain(ADS1220_GAIN_Values gain);                  // CONF0: Set the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available
  bool setMux(ADS1220_MUX_Values mux);                     // CONF0:
  ADS1220_MUX_Values getMux();                             // CONF0:
  bool setBurnoutCurrentSource(ADS1220_BCS_Values bo);     // CONF1: Burnout current sources
  bool setTemperatureSensorMode(ADS1220_TS_Values ts);     // CONF1: Temperature Sensor mode
  bool setConversionMode(ADS1220_CM_Values cm);            // CONF1: Conversion mode
  bool setOperatingMode(ADS1220_MODE_Values mode);         // CONF1: operating mode
  bool setDataRate(ADS1220_DR_Values dr);                  // CONF1: datarate
  bool setIdacCurrent(ADS1220_IDAC_Values idac);           // CONF2: IDAC current setting
  bool setPowerSwitch(ADS1220_PSW_Values psw);             // CONF2: power switch config
  bool setFirFilter(ADS1220_FIR_50_60_Values fir_mode);    // CONF2: FIR filter config
  bool setVoltageReference(ADS1220_VREF_Values vref);      // CONF2: Voltage reference selection
  bool setDataReadyMode(ADS1220_DRDYM_Values drdym);       // CONF3: Data ready mode
  bool setIDAC2Routing(ADS1220_I2MUX_Values i2mux);        // CONF3: IDAC2 routing config
  bool setIDAC1Routing(ADS1220_I1MUX_Values i1mux);        // CONF3: IDAC1 routing config

  // commands
  bool cmdStartSync();
  bool cmdReset();
  bool cmdPowerDown();

  // utility functions that complete the lib
  bool available();                                   // Returns true if DRDY Ready is set (conversion is complete)
  bool waitUntilAvailable(uint16_t timeout_ms = 100); // blocks until timeout (return false) or data avail within timeout (return true)
  int32_t getReading();                               // includes offset from internal calibration
  int32_t getAverageReading(uint8_t number = 4);      // get average corrected offset from internal calibration
  bool internalCalibration();                         // Call after mode changes
  bool sensorConnected();                             // Use burnout current to detect open connection (no sensor)
  void printRegisterValues();

public:
  // bool tara();

  /* *************************************************** PRIVATE *************************************************** */

private: // ADS1220 related stuff
  int32_t _internal_calibration_offset = 0;
  int32_t getReadingRaw();                          // straight from adc
  int32_t getAverageReadingRaw(uint8_t number = 4); // straight from adc but averaged
  void setInternalCalibrationOffset(int32_t val);
  int32_t getInternalCalibrationOffset();

private: // SPI and communication related stuff
  SPIClass *_spiPort;
  SPISettings _SPISettings;
  uint8_t _CS = ADS1220_CS_PIN_DEFAULT;
  uint8_t _DRDY = ADS1220_DRDY_PIN_DEFAULT;

  bool setBit(uint8_t bitNumber, uint8_t registerAddress);
  bool setBit(uint8_t bitNumber, uint8_t registerAddress, bool value);
  bool clearBit(uint8_t bitNumber, uint8_t registerAddress);
  bool getBit(uint8_t bitNumber, uint8_t registerAddress);
  uint8_t readRegister(uint8_t reg);
  bool writeRegister(uint8_t reg, uint8_t val);
  bool command(uint8_t cmd);
};
