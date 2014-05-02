/**
  ******************************************************************************
  * @file    ads1148.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include "task.h"
#include "ads1148.h"
#ifdef DEBUG
#include <stdio.h>
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t muxLookup[4] = {0x01, 0x13, 0x25, 0x37};
static const uint8_t idacMuxLookup[4] = {0x01, 0x23, 0x45, 0x67};
static const uint8_t nopbuf [4] = {0xFF, 0xFF, 0xFF, 0xFF}; // For RO operations write NOPs
static uint8_t txbuf [4];
static uint8_t rxbuf [4];
__IO int16_t latestConv[4];

#ifdef DEBUG
char buf[50];
#endif

static xSemaphoreHandle ADSSemaphore = NULL;
/* Private function prototypes -----------------------------------------------*/
void adsGPIOInit(void);
// SPI RW
void spiReadWrite(uint8_t *rbuf , const uint8_t *tbuf , int cnt);
// Run calibration commands
void adsCalib(int cal_cmd);
// Run calibration
void adsCalibrate(void);

/* Private functions ---------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
void spiInit()
{
  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

// SPI 1 on PA
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SPI1, DISABLE); // Do not remap

// SPI1 Pin initialization
  /* SPI SCK, MOSI, MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = ADS_MOSI_PIN | ADS_CLK_PIN /*| ADS_CS_PIN*/;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = ADS_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Common args for the SPI init structure
  SPI_StructInit (& SPI_InitStructure);	
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  // @ 48 MHz the prescaler must be at least 64 to meet the 500ns clk cycle min. for ADS1148
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	
  // SPI1 Master initialization
  SPI_Cmd(SPI1, DISABLE);
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_CalculateCRC(SPI1, DISABLE);
  SPI_Cmd(SPI1, ENABLE);
}

/* ---------------------------------------------------------------------------*/
void spiReadWrite(uint8_t *rbuf , const uint8_t *tbuf , int cnt)
{
	int i;
	
	for (i = 0; i < cnt; i++)
	{
		if (tbuf) 
		{
			SPI_I2S_SendData(SPI1 , *tbuf ++);
		} 
		else 
		{
			SPI_I2S_SendData(SPI1 , 0xff);
		}
		
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		if (rbuf) 
		{
			*rbuf++ = SPI_I2S_ReceiveData(SPI1);
		} 
		else 
		{
			SPI_I2S_ReceiveData(SPI1);
		}
	}
}

/* ---------------------------------------------------------------------------*/
void adsGPIOInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIOB Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  /* Configure the ADS_CS pin */
  GPIO_InitStructure.GPIO_Pin = ADS_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure the ADS_DRDY pin */
  GPIO_InitStructure.GPIO_Pin = ADS_DRDY_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure the ADS_START and ADS_RESET pins */
  GPIO_InitStructure.GPIO_Pin = ADS_START_PIN | ADS_RESET_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* ---------------------------------------------------------------------------*/
void adsIrqInit(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = ADS_EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_Init(&EXTI_InitStructure);

  GPIO_EXTILineConfig(ADS_EXTI_PORTSOURCE , ADS_DRDY_PINSOURCE);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; //Low prio
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0E; //Low prio
  NVIC_Init(&NVIC_InitStructure);

}

/* ---------------------------------------------------------------------------*/
void adsIrqDisable(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = ADS_EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

  EXTI_Init(&EXTI_InitStructure);
}

/* ---------------------------------------------------------------------------*/
void adsIrqEnable(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = ADS_EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

  /* Clear any pending interrupts */
  EXTI_ClearITPendingBit(ADS_EXTI_LINE);

  EXTI_Init(&EXTI_InitStructure);
}

/* ---------------------------------------------------------------------------*/
void adsCalib(int cal_cmd)
{
  uint8_t tx[3];
  uint8_t rx[3];
  // START needs to be high to enable all cmds
  // If START is deasserted only RDATA, RDATAC, SDATAC, WAKEUP and NOP can be issued.
  GPIO_SetBits(GPIOC, ADS_START_PIN);

  /*MUX1 reg is set for default values for CLKSTAT, VREFCON and REFSELT */
  if(ADS_CALIB_SYSOCAL == cal_cmd)
  {
    /* Initially set MUX1 for offset calibration */
    tx[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
    tx[1] = 0; // write 1 byte
    tx[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_OFFSET_MEASUREMENT );
    spiReadWrite(rx, tx, 3);

    tx[0] = ADS_SYSOCAL_Cmd;
    spiReadWrite(rx, tx, 1);
    while(adsGetDrdy() == Bit_SET) {} //await /DRDY
  }
  else if(ADS_CALIB_SYSGCAL == cal_cmd) 
  {
    /* Initially set MUX1 for offset calibration */
    tx[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
    tx[1] = 0; // write 1 byte
    tx[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_GAIN_MEASUREMENT );
    spiReadWrite(rx, tx, 3);

    tx[0] = ADS_SYSGCAL_Cmd;
    spiReadWrite(rx, tx, 1);
    while(adsGetDrdy() == Bit_SET) {} //await /DRDY
  }
  else if(ADS_CALIB_SELFOCAL == cal_cmd) 
  {
    /* No Mux needs to be set for this calibration */
    tx[0] = ADS_SELFOCAL_Cmd;
    spiReadWrite(rx, tx, 1);
    while(adsGetDrdy() == Bit_SET) {} //await /DRDY
  }
  /* Deassert START to stop free runing converting */
  GPIO_ResetBits(GPIOC, ADS_START_PIN);

  /* Finally set MUX1 for normal operation */
  tx[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_NORMOP );
  spiReadWrite(rx, tx, 3);

}
// Delay loop is about 6 instructions per loop. Clock is stored in: HSI_Value
void SSI_Delay(int n)
{
  volatile int SPI_DelayCount;
  for(SPI_DelayCount=0; SPI_DelayCount < n; SPI_DelayCount++)
  {
  }
}

/* ---------------------------------------------------------------------------*/
/* This function detects the presence of an ADS1148 by writhing an abitrary   */
/* value to some register and then retrieving it again. If the same bit       */
/* pattern is read back the ADS1148 is deemed present. If not an error event  */
/* issued on the modbus and further initialization is skipped.                */

// START needs to be high when this function is called
int adsDetectHW(void)
{
  uint8_t tx[3];
  uint8_t rx[3];
  int ret = TRUE;

  /* Switch on VREF */
  tx[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_ONBOARD << 3) | ADS_MUXCAL_NORMOP );
  spiReadWrite(rx, tx, 3);
  
  vTaskDelay(3); // Mockup uses 22uF => settel time 2 -3 ms

  /* Write something in to a register on the ADS1148 */
  tx[0] = ADS_WREG_Cmd | ADS_GPIOCFG_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = 0xA5;
  spiReadWrite(rx, tx, 3);

  /* Read back the written value from the ADS1148 */
  tx[0] = ADS_RREG_Cmd | ADS_GPIOCFG_Reg;
  tx[1] = 0; // read 1 byte
  spiReadWrite(rx, tx, 2);
  spiReadWrite(rx, nopbuf, 1);
  
  if (0xA5 != rx[0])
  {
    /* ADS failure means failure on both tubes */
    ret = FALSE;
#ifdef DEBUG
    gdi_send_msg_response("ADS detect failure\r\n");
#endif
  }

  /* Reset to analog input */
  tx[0] = ADS_WREG_Cmd | ADS_GPIOCFG_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = 0;
  spiReadWrite(rx, tx, 3);

  return ret;
}
/* ---------------------------------------------------------------------------*/
int adsDetectSensor(void)
{
#define FS 0x7F00  //32512
  int i, j; // iterator
  int16_t value;
  uint8_t tx[6];
  uint8_t rx[6];
  int ret = TRUE;

  /* Stop reading data continiously */
  tx[0] = ADS_RDATA_Cmd;
  spiReadWrite(rx, tx, 1);

  tx[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
  tx[1] = 3; // write 4 bytes: MUX0, VBIAS, MUX1, SYS0
  /* MUX0  : Burnout current on 10uA  */
  tx[2] = ( (ADS_BCS_SENSOR_DETECT << 6) | (muxLookup[0]) );
  /* VBIAS : Vbias off */
  tx[3] = 0;
  /* MUX1  : Internal OSC, Internal reference always on, Onboard Vref for ADC, MUXCAL norm */
  tx[4] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_ONBOARD << 3) | ADS_MUXCAL_NORMOP );
  /* SYS0  : PGA Gain = 4 (010b = 2d), Data Output Rate = 2000SPS */
  tx[5] = ( (2 << 4) | ADS_DOR );
  spiReadWrite(rx, tx, 6);

  vTaskDelay(3); // Mockup uses 22uF => settel time 2 - 3 ms for Vref

  // for each channel
  for(i = 0; i < 4; i++) {
    // Set MUX
    tx[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
    tx[1] = 0; // write 1 byte
    tx[2] = ( (ADS_BCS_SENSOR_DETECT << 6) | (muxLookup[i]) ); // Burnout current on 
    //GPIO_SetBits(GPIOB, ADS_START_PIN);
    spiReadWrite(rx, tx, 3);

    while(adsGetDrdy() == Bit_SET) {} //await /DRDY
    adsRead(&value);

#ifdef DEBUG
    sprintf(buf,"ADS sensor #%d detect: ADC val: 0x%08x", i, value);
    gdi_send_msg_response(buf);
#endif

    if( (FS < value)/* open circuit */ /*|| (-FS > value) short circuit */) {
      if(2 > i) {
        //setStatusReg(INIT_HW_ERROR_TUBE1);
      } else {
        //setStatusReg(INIT_HW_ERROR_TUBE2);
      }
      ret = FALSE;
    }
  }
}
/* ---------------------------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/
uint16_t dac_2_temp(signed short dac)
{
	int64_t res;
  res = (((dac*29549)/10000)+77175)/100;
  return (uint16_t)res;
}

/* ---------------------------------------------------------------------------*/
signed short temp_2_dac(int16_t temp)
{
	int64_t res;
  res = (10000*(((int64_t)temp*100)-77175))/29549;
  return (signed short)res;
}

/* ---------------------------------------------------------------------------*/
int ads1148Init(void)
{
  uint8_t tx[3];
  uint8_t rx[3];
  int ret = 0;

  adsGPIOInit();
  GPIO_ResetBits(GPIOC, ADS_RESET_PIN);
  // Keep ADS1148 reset active for 1,1us minimun using internal osc. Spend waiting time initialasing SPI.
  spiInit();
  GPIO_SetBits(GPIOC, ADS_RESET_PIN);
  GPIO_ResetBits(GPIOA, ADS_CS_PIN); // Activate Chip Select pin

  // START needs to be high to enable all cmds
  // If START is deasserted only RDATA, RDATAC, SDATAC, WAKEUP and NOP can be issued.
  GPIO_SetBits(GPIOC, ADS_START_PIN);

  if(adsDetectHW())
  {
    adsDetectSensor();

    /* Reset */
    tx[0] = ADS_RESET_Cmd;
    spiReadWrite(rx, tx, 1);
    vTaskDelay(1);
    
    /* Switch on VREF */
    tx[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
    tx[1] = 0; // write 1 byte
    tx[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_NORMOP );
    spiReadWrite(rx, tx, 3);

    vTaskDelay(3); // Mockup uses 22uF => settel time 2 -3 ms

    /* MUX0 Defaults to ch 1 which is OK */
    /* PGA Gain and Data Output Rate */
    tx[0] = ADS_WREG_Cmd | ADS_SYS0_Reg;
    tx[1] = 0; // write 1 byte
    tx[2] = ( (ADS_PGA << 4) |(ADS_DOR) );
    spiReadWrite(rx, tx, 3);
    /* DOUT is only DOUT (DRDY is DRDY pin), IADC magnitude and IDAC routing */
    tx[0] = ADS_WREG_Cmd | ADS_IDAC0_Reg;
    tx[1] = 1; // write 2 byte
    tx[2] = (ADS_IMAG) ;
    tx[3] = ( idacMuxLookup[0] );
    spiReadWrite(rx, tx, 4);
    
    /* Deassert START to stop free runing converting */
    GPIO_ResetBits(GPIOC, ADS_START_PIN);
    adsCalibrate();
    adsIrqInit();
  }
  else
  {
    ret = -1;
  }
  return ret;
}

/* ---------------------------------------------------------------------------*/
uint8_t adsGetDrdy(void)
{
  return GPIO_ReadInputDataBit(GPIOB, ADS_DRDY_PIN);
  //Bit_SET, Bit_RESET
}

/* ---------------------------------------------------------------------------*/
void adsCalibrate(void)
{
//  adsCalib(ADS_CALIB_SYSOCAL);
//  DEBUG_String("ADS1148 SYSOCAL, ");
  adsCalib(ADS_CALIB_SYSGCAL);
//  DEBUG_String("SYSGCAL, ");
  adsCalib(ADS_CALIB_SELFOCAL);
//  DEBUG_String("SELFOCAL\r\n");
}

/* ---------------------------------------------------------------------------*/
void adsStepCalibrate(void)
{
  static uint8_t calibrationStep = 0;
  switch(calibrationStep)
  {
    case 0:
    {
      calibrationStep = 1;
    }
    break;
    case 1:
    {
      adsCalib(ADS_CALIB_SYSOCAL);
      calibrationStep = 2;
    }
    break;
    case 2:
    {
      adsCalib(ADS_CALIB_SYSGCAL);
      adsCalib(ADS_CALIB_SELFOCAL);
      calibrationStep = 3;
    }
    break;
    default:
    {
      calibrationStep = 0;        
    }
  break;
  }
}

/* ---------------------------------------------------------------------------*/
void adsReadCalib(uint32_t * offsetcal, uint32_t * fullscalecal)
{
  uint8_t tx[3];
  uint8_t rx[3];
  const uint8_t nopbuf [4] = {0xFF, 0xFF, 0xFF, 0xFF}; // For RO operations write NOPs
  // If START is deasserted only RDATA, RDATAC, SDATAC, WAKEUP and NOP can be issued.
  GPIO_SetBits(GPIOC, ADS_START_PIN);

  /* Read OFC0, OFC1 and OFC2 in one go */
  tx[0] = ADS_RREG_Cmd | ADS_OFC0_Reg;
  tx[1] = 2; // read 3 bytes
  spiReadWrite(rx, tx, 2);
  /* Read values from OFC reg  */
  spiReadWrite(rx, nopbuf, 3); // Read result
  *offsetcal = 0; //clear 8 msb
  *offsetcal = (rx[2] << 16) + (rx[1] << 8) + rx[0]; 

  /* Read FSC0, FSC1 and FSC2 in one go */
  tx[0] = ADS_RREG_Cmd | ADS_FSC0_Reg;
  tx[1] = 2; // read 3 bytes
  spiReadWrite(rx, tx, 2);
  /* Read values from OFC reg  */
  spiReadWrite(rx, nopbuf, 3); // Read result
  *fullscalecal = 0; //clear 8 msb
  *fullscalecal = (rx[2] << 16) + (rx[1] << 8) + rx[0]; 

  /* Deassert START to stop free runing converting */
  GPIO_ResetBits(GPIOC, ADS_START_PIN);

}
  
  /* ---------------------------------------------------------------------------*/
void adsStart(const uint8_t ch)
{
  uint8_t tx[3];
  uint8_t rx[3];
  if(4 > ch)
  {
    /* Route IDAC current */
    tx[0] = ADS_WREG_Cmd | ADS_IDAC1_Reg;
    tx[1] = 0; // write 1 byte
    tx[2] = ( idacMuxLookup[ch] );
    spiReadWrite(rx, tx, 3);

    /* Initially set MUX0 */
    tx[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
    tx[1] = 0; // write 1 byte
    tx[2] = ( (ADS_BCS << 6) | (muxLookup[ch]) );
    spiReadWrite(rx, tx, 3);

    /* Pulse the START pin */
    GPIO_SetBits(GPIOC, ADS_START_PIN);
    SSI_Delay(10); // a wait of 80us minimun using internal osc
    GPIO_ResetBits(GPIOC, ADS_START_PIN);
  }
}

/* ---------------------------------------------------------------------------*/
void adsRead(int16_t * value)
{
  uint8_t tx[3];
  uint8_t rx[3];
  const uint8_t nopbuf [4] = {0xFF, 0xFF, 0xFF, 0xFF}; // For RO operations write NOPs
  if(Bit_RESET == adsGetDrdy())
    { // if DRDY is low there are data to read.
     /* Read values from ADC  */
      tx[0] = ADS_RDATA_Cmd;
      spiReadWrite(rx, tx, 1);    // Send command
      spiReadWrite(rx, nopbuf, 2); // Read result (Writing NOPs)
      *value = (rx[0] << 8) + rx[1]; 
    }
}

/* ---------------------------------------------------------------------------*/
void adsStartSeq(void)
{
  uint8_t tx[3];
  uint8_t rx[3];
  // RDATAC mode is required!!

  adsIrqInit(); //Initialize the IRQ system.
  // Start converting on ch 0.  
  /* Route IDAC current */
  tx[0] = ADS_WREG_Cmd | ADS_IDAC1_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = ( idacMuxLookup[0] );
  spiReadWrite(rx, tx, 3);
  
  // Start ReadDataContinious mode.
  tx[0] = ADS_RDATAC_Cmd;
  spiReadWrite(rx, tx, 1);

  /* Initially set MUX0 */
  tx[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = ( (ADS_BCS << 6) | (muxLookup[0]) );
  spiReadWrite(rx, tx, 3);
  
  /* SET the START pin */
  GPIO_SetBits(GPIOC, ADS_START_PIN);
  adsIrqEnable();
}

/* ---------------------------------------------------------------------------*/
void adsTimerCallback(xTimerHandle xTimer)
{
  adsIrqEnable();
  GPIO_SetBits(GPIOC, ADS_START_PIN);
/** #### DEBUG */
//  xSemaphoreGiveFromISR(ADSSemaphore, pdFALSE);
/* #### DEBUG END*/
}

void adsContiniueSeq(void)
{
  adsIrqEnable();
  GPIO_SetBits(GPIOC, ADS_START_PIN);
}

/* ---------------------------------------------------------------------------*/
void adsStoptSeq(void)
{ 
  uint8_t tx[3];
  uint8_t rx[3];
  //De-assert START
  GPIO_ResetBits(GPIOC, ADS_START_PIN);

  // Stop ReadDataContinious mode.
  tx[0] = ADS_SDATAC_Cmd;
  spiReadWrite(rx, tx, 1);

  // disable /DRDY irq? (or set a flag indicating this for next and last /DRDY irq)
}

/* ---------------------------------------------------------------------------*/
void adsGetLatest(int16_t * ch0value, int16_t * ch1value, int16_t * ch2value, int16_t * ch3value)
{
  taskENTER_CRITICAL(); //push irq state
  *ch0value = latestConv[0];
  *ch1value = latestConv[1];
  *ch2value = latestConv[2];
  *ch3value = latestConv[3];
  taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------*/
void adsConfigConversionTimer(tmrTIMER_CALLBACK convStartFn)
{
  xTimerHandle xTimer;
  signed portBASE_TYPE * pxReschedule = pdFALSE;
  xTimer= xTimerCreate("ADCTimer",      // Just a text name, not used by the kernel.
                       ((configTICK_RATE_HZ)/SAMPLING_FREQUENCY),  // conversion frequency.
                       pdTRUE,          // The timers will auto-reload themselves when they expire.
                       (void *) 1,      // Assign each timer a unique id equal to its array index.
                       convStartFn      // Each timer calls the same callback when it expires.
                       );
                           
  if( xTimer == NULL )
  {
    // The timer was not created.
  }
  else
  {
    // Start the timer.  No block time is specified, and even if one was
    // it would be ignored because the scheduler has not yet been
    // started.
    if( xTimerStart( xTimer, 0 ) != pdPASS )
    {
      // The timer could not be set into the Active state.
    }
  }
}

void adsSetIsrSemaphore(xSemaphoreHandle sem)
{
  ADSSemaphore = sem;
}

/* ---------------------------------------------------------------------------*/
static uint16_t readAndPrepareNext(uint8_t channel)
{
  uint8_t tx[3];
  uint8_t rx[3];
  uint16_t value;
  
  /* Route IDAC current */
  tx[0] = ADS_WREG_Cmd | ADS_IDAC1_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = ( idacMuxLookup[(channel+1)%4] );
  spiReadWrite(rx, tx, 3);
  //result of just completed conversion is in rxbuf
  value = (rx[0] << 8) + rx[1]; //
  
  /* Initially set MUX0 */
  tx[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
  tx[1] = 0; // write 1 byte
  tx[2] = ( (ADS_BCS << 6) | (muxLookup[(channel+1)%4]) );
  spiReadWrite(rx, tx, 3);

  return value;
}

/* ---------------------------------------------------------------------------*/
/* Sequential ADC is starte by calling adsStartSeq() which starts the first conversion on ch 0  */
/*                                                                                                                                   */
void ADS_Handler(void)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
#if 1
	typedef enum {
    CHANNEL_0_SAMPLED,
    CHANNEL_1_SAMPLED,
    CHANNEL_2_SAMPLED,
    CHANNEL_3_SAMPLED,
  } state_t;
  
  static state_t state = CHANNEL_0_SAMPLED;
  
  //use privete rxbuf and txbuf? - As long as "spiReadWrite" is used - no.
  // RDATAC mode is required!!
  // NB! SPI on M3 has no FIFO -> This causes busy waiting for 6 byts transmit time!!
  // The SPI activities in this function is measured at approx 90 us.

  switch (state) {
    case CHANNEL_0_SAMPLED:
      latestConv[0] = readAndPrepareNext(0);
      state++;
      break;
    case CHANNEL_1_SAMPLED:
      latestConv[1] = readAndPrepareNext(1);
      state++;
      break;
    case CHANNEL_2_SAMPLED:
      latestConv[2] = readAndPrepareNext(2);
      state++;
      break;
    case CHANNEL_3_SAMPLED:
      latestConv[3] = readAndPrepareNext(3);
      GPIO_ResetBits(GPIOC, ADS_START_PIN);
      adsIrqDisable();
  /* Synchronize adsConfigConversionTimer. Do not require context switch in case
     running task is lower prio adsConfigConversionTimer (pdTRUE to do so)*/
      xSemaphoreGiveFromISR(ADSSemaphore, &xHigherPriorityTaskWoken); 
      state = CHANNEL_0_SAMPLED;
      break;
  }
#else
  xSemaphoreGiveFromISR(ADSSemaphore, pdFALSE);
#endif
  EXTI_ClearITPendingBit(ADS_EXTI_LINE);
  //portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

