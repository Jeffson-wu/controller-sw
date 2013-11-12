/**
  ******************************************************************************
  * @file    ads1148.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    9-Sep -2013
  * @brief   Header for ADS1148 chip
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 Xtel </center></h2>
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADS1148_H
#define __ADS1148_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* BSC Burnout current, 0 = off */
#define ADS_BCS 0
/* PGA gain is 2^ADS_PGA; E.g. ADS_PGA = 4 => x16 amplification */
/* x16 used for 4w, and x32 for 3w (x32 => ADS_PGA = 5) */
#define ADS_PGA 6
/* Data Output Rate */
/* 0 => 5 SPS, 1 => 10 SPS, 2 => 20 SPS, .. 5 => 160 SPS, ... 8 => 1000SPS and 9 to 15 all => 2000SPS */
#define ADS_DOR 10
/* IMAG Magnitude of excitation current 0 = off, 1= 50uA, 2 = 100uA, ... */
#define ADS_IMAG 2
/* I1DIR IDAC1 current routing */
#define ADS_I1DIR 8
/* I2DIR IDAC2 current routing */
#define ADS_I2DIR  0x0F
/* MUX1 MUXCAL calibration routing */
#define ADS_CLKSTAT_INTERNAL_OSC 0
#define ADS_CLKSTAT_EXTERNAL_OSC 1
#define ADS_VREFCON 1
#define ADS_REFSELT_REF0 0
#define ADS_MUXCAL_NORMOP 0
#define ADS_MUXCAL_OFFSET_MEASUREMENT 1
#define ADS_MUXCAL_GAIN_MEASUREMENT 2

#define ADS_CALIB_SYSOCAL 0
#define ADS_CALIB_SYSGCAL 1
#define ADS_CALIB_SELFOCAL 2

#define ADS_START_PIN GPIO_Pin_2 //PC2
#define ADS_RESET_PIN GPIO_Pin_1 //PC1
#define ADS_DRDY_PIN GPIO_Pin_10 //PB10
#define ADS_EXTI_LINE EXTI_Line10
#define ADS_DRDY_PINSOURCE GPIO_PinSource10
#define ADS_EXTI_PORTSOURCE GPIO_PortSourceGPIOB

#define ADS_MISO_PIN GPIO_Pin_6
#define ADS_MOSI_PIN GPIO_Pin_7
#define ADS_CLK_PIN GPIO_Pin_5
#define ADS_CS_PIN GPIO_Pin_4
#define ADS_MISO_PinSource GPIO_PinSource6
#define ADS_MOSI_PinSource GPIO_PinSource7
#define ADS_CLK_PinSource GPIO_PinSource5
#define ADS_CS_PinSource GPIO_PinSource4

/* Exported constants --------------------------------------------------------*/

/** @defgroup SPI_Exported_Constants
  * @{
  */

// Addressed
#define ADS_MUX0_Reg        ((uint8_t)0x00)
#define ADS_VBIAS_Reg       ((uint8_t)0x01)
#define ADS_MUX1_Reg        ((uint8_t)0x02)
#define ADS_SYS0_Reg        ((uint8_t)0x03)
#define ADS_OFC0_Reg        ((uint8_t)0x04)
#define ADS_OFC1_Reg        ((uint8_t)0x05)
#define ADS_OFC2_Reg        ((uint8_t)0x06)
#define ADS_FSC0_Reg        ((uint8_t)0x07)
#define ADS_FSC1_Reg        ((uint8_t)0x08)
#define ADS_FSC2_Reg        ((uint8_t)0x09)
#define ADS_IDAC0_Reg       ((uint8_t)0x0A)
#define ADS_IDAC1_Reg       ((uint8_t)0x0B)
#define ADS_GPIOCFG_Reg     ((uint8_t)0x0C)
#define ADS_GPIODIR_Reg     ((uint8_t)0x0D)
#define ADS_GPIODAT_Reg     ((uint8_t)0x0E)

// Commands
#define ADS_WAKEUP_Cmd      ((uint8_t)0x00)
#define ADS_SLEEP_Cmd       ((uint8_t)0x02)
#define ADS_SYNC_Cmd        ((uint8_t)0x04)
#define ADS_RESET_Cmd       ((uint8_t)0x06)
#define ADS_NOP_Cmd         ((uint8_t)0xFF)
#define ADS_RDATA_Cmd       ((uint8_t)0x12)
#define ADS_RDATAC_Cmd      ((uint8_t)0x14)
#define ADS_SDATAC_Cmd      ((uint8_t)0x16)
#define ADS_RREG_Cmd        ((uint8_t)0x20)
#define ADS_WREG_Cmd        ((uint8_t)0x40)
#define ADS_SYSOCAL_Cmd     ((uint8_t)0x60)
#define ADS_SYSGCAL_Cmd     ((uint8_t)0x61)
#define ADS_SELFOCAL_Cmd    ((uint8_t)0x62)


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

// Set up as we use it.
extern void ads1148Init(void);
// Get state of DRDY
extern uint8_t adsGetDrdy(void);
// Start conversion on one of the four channels
extern void adsStart(const uint8_t ch);
// Read last conversion
extern void adsRead(int16_t * value);
// Start sequential convertion all four ch.
extern void adsStartSeq(void);
// Re-start conversion
extern void adsContiniueSeq(void);

extern void adsTimerCallback(xTimerHandle xTimer);
// Stop sequential convertion.
extern void adsStoptSeq(void);
// Retrieve latest value from all ADC channels.
extern void adsGetLatest(int16_t * ch0value, int16_t * ch1value, int16_t * ch2value, int16_t * ch3value);

extern void adsConfigConversionTimer(tmrTIMER_CALLBACK convStartFn);


#endif /* __ADS1148_H */

/************************ (C) COPYRIGHT Xtel *****END OF FILE****/

