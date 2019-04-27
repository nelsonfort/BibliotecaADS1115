/*
 * ADS1115.c
 *
 *  Created on: 19/4/2019
 *      Author: nelsonf
 */
/*------------Basado en la biblioteca "Driver for the----------------*/
/*-------ADS1015/ADS1115 ADC - K.Townsend (Adafruit Industries)"-----*/
/*--------------- Modificado por Nelson Fortunatti ------------------*/

#include "ADS1115.h"


/*Funciones I2C
bool_t i2cInit( i2cMap_t i2cNumber, uint32_t clockRateHz );

bool_t i2cRead( i2cMap_t  i2cNumber,
                uint8_t  i2cSlaveAddress,
                uint8_t* dataToReadBuffer,
                uint16_t dataToReadBufferSize,
                bool_t   sendWriteStop,
                uint8_t* receiveDataBuffer,
                uint16_t receiveDataBufferSize,
                bool_t   sendReadStop );
    i2cRead( I2C0,control.address,&subAddress,1,TRUE,control._buffer,count,TRUE)

bool_t i2cWrite( i2cMap_t  i2cNumber,
                 uint8_t  i2cSlaveAddress,
                 uint8_t* transmitDataBuffer,
                 uint16_t transmitDataBufferSize,
                 bool_t   sendWriteStop );

    uint8_t transmitDataBuffer[2];
	transmitDataBuffer[0] = subAddress;
	transmitDataBuffer[1] = data;
	i2cWrite(I2C0, control.address, transmitDataBuffer, 2, TRUE);
*/
uint8_t	beginConnection() {
    return i2cInit(I2C0, ADS1115_CLOCKRATE );
}

uint8_t writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
	uint8_t transmitDataBuffer[3];

	if( (reg <0) || (reg>3) ){
		return -1;
	}

	transmitDataBuffer[0] = reg;
	transmitDataBuffer[1] = (uint8_t) (value >> 8);
	transmitDataBuffer[2] = value & 0xff ;
/* Structure i2cWrite
 * bool_t i2cWrite( i2cMap_t  i2cNumber,
                 uint8_t  i2cSlaveAddress,
                 uint8_t* transmitDataBuffer,
                 uint16_t transmitDataBufferSize,
                 bool_t   sendWriteStop );
 */
	if( i2cWrite(I2C0, i2cAddress, transmitDataBuffer, 3, TRUE) )
		return 3;
	else
		return -1;

}

static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) {
	uint8_t bufferRead[2];
	uint16_t result;

	/* Structure i2cRead
	 * bool_t i2cRead( i2cMap_t  i2cNumber,
                uint8_t  i2cSlaveAddress,
                uint8_t* dataToReadBuffer,
                uint16_t dataToReadBufferSize,
                bool_t   sendWriteStop,
                uint8_t* receiveDataBuffer,
                uint16_t receiveDataBufferSize,
                bool_t   sendReadStop );
	 */

	if( i2cRead( I2C0, i2cAddress, &reg,1,TRUE,bufferRead,2,TRUE) )
		result = (bufferRead[0] << 8) | bufferRead[1];
	else
		result = 0;

	return result;
}

void setADC_ADDRESS(uint8_t i2cAddress)
{
   m_i2cAddress = i2cAddress;
}


void setGain(adsGain_t gain)
{
    m_gain = gain;
}

adsGain_t getGain()
{
  return m_gain;
}

uint16_t readADC_SingleEnded(uint8_t channel) {

    if (channel > 3)
    {
        return 0;
    }

    // Start with default values
    uint16_t config = ADS1115_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1115_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

    /*uint16_t config = ADS1115_REG_CONFIG_CQUE_2CONV    | // Disable the comparator (default val)
    					ADS1115_REG_CONFIG_CLAT_LATCH   | // Non-latching (default val)
                        ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                        ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                        ADS1115_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                        ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
    */
    // Set PGA/voltage range
    config |= m_gain;

    // Set single-ended input channel
    switch (channel)
    {
        case (0):
          config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
          break;
        case (1):
          config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
          break;
        case (2):
          config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
          break;
        case (3):
          config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
          break;
    }

    // Set 'start single-conversion' bit
    config |= ADS1115_REG_CONFIG_OS_SINGLE;

    // Write config register to the ADC
    writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);

    // Wait for the conversion to complete
    delayInaccurateUs(ADS1115_CONVERSIONDELAY*2);

    // Read the conversion results
    // Shift 12-bit results right 4 bits for the ADS1115
    return readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT);
}

//--------------- Not implemented yet-------------------------
/*
int16_t readADC_Differential() {

}
*/
void startComparator_SingleEnded(uint8_t channel, int16_t threshold)
{
  // Start with default values
  uint16_t config = ADS1115_REG_CONFIG_CQUE_1CONV   | // Comparator enabled and asserts on 1 match
                    ADS1115_REG_CONFIG_CLAT_LATCH   | // Latching mode
                    ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1115_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1115_REG_CONFIG_MODE_CONTIN  | // Continuous conversion mode
                    ADS1115_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1115
  writeRegister(m_i2cAddress, ADS1115_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);
}


int16_t getLastConversionResults()
{
  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res = readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0)
  {
    return (int16_t)res;
  }
  else
  {
    // Shift 12-bit results right 4 bits for the ADS1115,
    // making sure we keep the sign bit intact
    if (res > 0x07FF)
    {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}
//-- The PT100 are 2 wire devices measured in single shot-mode, no diferential mode.
//-- These functions ARE ONLY SUITABLE for reading the thermistors PT100 by applying 5V DC voltage
//-- With a resistor of 3.3Kohm in series to the PT100, the voltage measures is the voltage in the PT100

float  getLastConvertion_voltage() //-- Only for thermistors PT100 connected to the ADC
{
	const float VPS = 0.256 / 32768.0; //-- Resolución en Volts
	//-- La resolución del ADC es de 15 bits el bit 16 es del signo.
	return getLastConversionResults() * VPS;
}

float  getLastConvertion_resistance() //-- Only for thermistors PT100 connected to the ADC
{
	 float convVolt;
	 convVolt = getLastConvertion_voltage();
	 return (3300 * convVolt ) / (5 - convVolt);
}

float  getLastConvertion_temperature() //-- Only for thermistors PT100 connected to the ADC
{
	return 2.597 * getLastConvertion_resistance() - 259.7;

}
