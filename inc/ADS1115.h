/*
 * ADS1115.h
 *
 *  Created on: 19/4/2019
 *      Author: nelsonf
 */
/*------------Basado en la biblioteca "Driver for the----------------*/
/*-------ADS1015/ADS1115 ADC - K.Townsend (Adafruit Industries)"-----*/
/*--------------- Modificado por Nelson Fortunatti ------------------*/
#ifndef ADS1115_H
#define ADS1115_H

#include "sapi.h"
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS1115_I2C_ADDRESS_GND             (0x48)    // 100 1000 (ADDR = GND)
    #define ADS1115_I2C_ADDRESS_VDD             (0x49)    // 100 1001 (ADDR = VDD)
    #define ADS1115_I2C_ADDRESS_SDA             (0x4A)    // 100 1010 (ADDR = SDA)
    #define ADS1115_I2C_ADDRESS_SCL             (0x4B)    // 100 1011 (ADDR = SCL)
/*=========================================================================*/

/*=========================================================================
    CLOCK RATE I2C CONNECTION
    -----------------------------------------------------------------------*/
    #define ADS1115_CLOCKRATE        (400000)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
    #define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1115_REG_POINTER_MASK        (0x03)
    #define ADS1115_REG_POINTER_CONVERT     (0x00)
    #define ADS1115_REG_POINTER_CONFIG      (0x01)
    #define ADS1115_REG_POINTER_LOWTHRESH   (0x02)
    #define ADS1115_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    REGISTER ADDRESS
    -----------------------------------------------------------------------*/
    #define ADDRESS_CONVERSION_REG         	(0)
	#define ADDRESS_CONFIG_REG         		(1)
/*=========================================================================*/


/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS1115_REG_CONFIG_OS_MASK      (0x8000)
    #define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS1115_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
    #define ADS1115_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS1115_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

    #define ADS1115_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
    #define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

    #define ADS1115_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
    #define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS1115_REG_CONFIG_DR_MASK      (0x00E0)
    #define ADS1115_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
    #define ADS1115_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
    #define ADS1115_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
    #define ADS1115_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second
    #define ADS1115_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
    #define ADS1115_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
    #define ADS1115_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second

    #define ADS1115_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
    #define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS1115_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
    #define ADS1115_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS1115_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
    #define ADS1115_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS1115_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS1115_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
    #define ADS1115_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS1115_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS1115_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/
    typedef enum
    {
        GAIN_6144    = ADS1115_REG_CONFIG_PGA_6_144V,
        GAIN_4096          = ADS1115_REG_CONFIG_PGA_4_096V,
        GAIN_2048          = ADS1115_REG_CONFIG_PGA_2_048V,
        GAIN_1024        = ADS1115_REG_CONFIG_PGA_1_024V,
        GAIN_0512       = ADS1115_REG_CONFIG_PGA_0_512V,
        GAIN_0256     = ADS1115_REG_CONFIG_PGA_0_256V
    } adsGain_t;

    uint8_t   m_i2cAddress;
    uint8_t   m_conversionDelay;
    uint8_t   m_bitShift;
    adsGain_t m_gain;

  uint8_t	beginConnection();
  void 		setADC_ADDRESS(uint8_t i2cAddress);
  void 		begin(void);
  uint16_t  readADC_SingleEnded(uint8_t channel);

  //int16_t   readADC_Differential_0_1(void);   //--Not implemented yet
  //int16_t   readADC_Differential_2_3(void);   //--Not implemented yet

  void      startComparator_SingleEnded(uint8_t channel, int16_t threshold);
  int16_t   getLastConversionResults();
  void      setGain(adsGain_t gain);
  adsGain_t getGain(void);

  float  	getLastConvertion_voltage();
  float  	getLastConvertion_resistance();
  float  	getLastConvertion_temperature();

#endif /* ADS1115_H */
