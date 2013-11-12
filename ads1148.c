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
#include "task.h"
#include "ads1148.h"

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

/* Private function prototypes -----------------------------------------------*/
void ADS_GPIOInit(void);
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
  GPIO_InitStructure.GPIO_Pin = ADS_MOSI_PIN | ADS_CLK_PIN | ADS_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = ADS_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
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
void ADS_GPIOInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIOB Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

  /* Configure the ADS_DRDY pin */
  GPIO_InitStructure.GPIO_Pin = ADS_DRDY_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure the ADS_START and ADS_RESET pins */
  GPIO_InitStructure.GPIO_Pin = ADS_RESET_PIN | ADS_START_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* ---------------------------------------------------------------------------*/
void adsIrqInit(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = ADS_EXTI_LINE;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;

  GPIO_EXTILineConfig(ADS_EXTI_PORTSOURCE , ADS_DRDY_PINSOURCE);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; //Low prio
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0E; //Low prio

  EXTI_Init(&EXTI_InitStructure);
  NVIC_Init(&NVIC_InitStructure);

}

/* ---------------------------------------------------------------------------*/
void adsCalib(int cal_cmd)
{
  // START needs to be high to enable all cmds
  // If START is deasserted only RDATA, RDATAC, SDATAC, WAKEUP and NOP can be issued.
  GPIO_SetBits(GPIOB, ADS_START_PIN);

  /*MUX1 reg is set for default values for CLKSTAT, VREFCON and REFSELT */
  if(ADS_CALIB_SYSOCAL == cal_cmd)
  {
    /* Initially set MUX1 for offset calibration */
    txbuf[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
    txbuf[1] = 0; // write 1 byte
    txbuf[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_OFFSET_MEASUREMENT );
    spiReadWrite(rxbuf, txbuf, 3);

    txbuf[0] = ADS_SYSOCAL_Cmd;
    spiReadWrite(rxbuf, txbuf, 1);
    while(adsGetDrdy() == Bit_SET) {} //await /DRDY
  }
  else if(ADS_CALIB_SYSGCAL == cal_cmd) 
  {
    /* Initially set MUX1 for offset calibration */
    txbuf[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
    txbuf[1] = 0; // write 1 byte
    txbuf[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_GAIN_MEASUREMENT );
    spiReadWrite(rxbuf, txbuf, 3);

    txbuf[0] = ADS_SYSGCAL_Cmd;
    spiReadWrite(rxbuf, txbuf, 1);
    while(adsGetDrdy() == Bit_SET) {} //await /DRDY
  }
  else if(ADS_CALIB_SELFOCAL == cal_cmd) 
  {
    /* No Mux needs to be set for this calibration */
    txbuf[0] = ADS_SELFOCAL_Cmd;
    spiReadWrite(rxbuf, txbuf, 1);
    while(adsGetDrdy() == Bit_SET) {} //await /DRDY
  }
  /* Deassert START to stop free runing converting */
  GPIO_ResetBits(GPIOB, ADS_START_PIN);

  /* Finally set MUX1 for normal operation */
  txbuf[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
  txbuf[1] = 0; // write 1 byte
  txbuf[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_NORMOP );
  spiReadWrite(rxbuf, txbuf, 3);

}
// Delay loop is about 6 instructions per loop.
void SSI_Delay(int n)
{
  volatile int SPI_DelayCount;
  for(SPI_DelayCount=0; SPI_DelayCount < n; SPI_DelayCount++)
  {
  }
}

/* Public functions ---------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
void ads1148Init(void)
{
  ADS_GPIOInit();
  GPIO_ResetBits(GPIOB, ADS_RESET_PIN);
  // Keep ADS1148 reset active for 1,1us minimun using internal osc. Spend waiting time initialasing SPI.
  spiInit();
  GPIO_SetBits(GPIOB, ADS_RESET_PIN);
  
  // START needs to be high to enable all cmds
  // If START is deasserted only RDATA, RDATAC, SDATAC, WAKEUP and NOP can be issued.
  GPIO_SetBits(GPIOB, ADS_START_PIN);

  /* Switch on VREF */
  txbuf[0] = ADS_WREG_Cmd | ADS_MUX1_Reg;
  txbuf[1] = 0; // write 1 byte
  txbuf[2] = ((ADS_CLKSTAT_INTERNAL_OSC << 7) | (ADS_VREFCON << 5) | (ADS_REFSELT_REF0 << 3) | ADS_MUXCAL_NORMOP );
  spiReadWrite(rxbuf, txbuf, 3);

  // ######## 75Mhz / 6 cycles.
  SSI_Delay(15000); // Mockup uses 22uF => settel time 2 -3 ms

  /* MUX0 Defaults to ch 1 which is OK */
  /* PGA Gain and Data Output Rate */
  txbuf[0] = ADS_WREG_Cmd | ADS_SYS0_Reg;
  txbuf[1] = 0; // write 1 byte
  txbuf[2] = ( (ADS_PGA << 4) |(ADS_DOR) );
  spiReadWrite(rxbuf, txbuf, 3);
  /* DOUT is only DOUT (DRDY is DRDY pin), IADC magnitude and IDAC routing */
  txbuf[0] = ADS_WREG_Cmd | ADS_IDAC0_Reg;
  txbuf[1] = 1; // write 2 byte
  txbuf[2] = (ADS_IMAG) ;
  txbuf[3] = ( idacMuxLookup[0] );
  spiReadWrite(rxbuf, txbuf, 4);
  
  /* Deassert START to stop free runing converting */
  GPIO_ResetBits(GPIOB, ADS_START_PIN);
  adsCalibrate();
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
  // If START is deasserted only RDATA, RDATAC, SDATAC, WAKEUP and NOP can be issued.
  GPIO_SetBits(GPIOB, ADS_START_PIN);

  /* Read OFC0, OFC1 and OFC2 in one go */
  txbuf[0] = ADS_RREG_Cmd | ADS_OFC0_Reg;
  txbuf[1] = 2; // read 3 bytes
  spiReadWrite(rxbuf, txbuf, 2);
  /* Read values from OFC reg  */
  spiReadWrite(rxbuf, nopbuf, 3); // Read result
  *offsetcal = 0; //clear 8 msb
  *offsetcal = (rxbuf[2] << 16) + (rxbuf[1] << 8) + rxbuf[0]; 

  /* Read FSC0, FSC1 and FSC2 in one go */
  txbuf[0] = ADS_RREG_Cmd | ADS_FSC0_Reg;
  txbuf[1] = 2; // read 3 bytes
  spiReadWrite(rxbuf, txbuf, 2);
  /* Read values from OFC reg  */
  spiReadWrite(rxbuf, nopbuf, 3); // Read result
  *fullscalecal = 0; //clear 8 msb
  *fullscalecal = (rxbuf[2] << 16) + (rxbuf[1] << 8) + rxbuf[0]; 

  /* Deassert START to stop free runing converting */
  GPIO_ResetBits(GPIOB, ADS_START_PIN);

}
  
  /* ---------------------------------------------------------------------------*/
void adsStart(const uint8_t ch)
{
  if(4 > ch)
  {
    /* Route IDAC current */
    txbuf[0] = ADS_WREG_Cmd | ADS_IDAC1_Reg;
    txbuf[1] = 0; // write 1 byte
    txbuf[2] = ( idacMuxLookup[ch] );
    spiReadWrite(rxbuf, txbuf, 3);

    /* Initially set MUX0 */
    txbuf[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
    txbuf[1] = 0; // write 1 byte
    txbuf[2] = ( (ADS_BCS << 6) | (muxLookup[ch]) );
    spiReadWrite(rxbuf, txbuf, 3);

    /* Pulse the START pin */
    GPIO_SetBits(GPIOB, ADS_START_PIN);
    SSI_Delay(10); // a wait of 80us minimun using internal osc
    GPIO_ResetBits(GPIOB, ADS_START_PIN);
  }
}

/* ---------------------------------------------------------------------------*/
void adsRead(int16_t * value)
{
  if(Bit_RESET == adsGetDrdy())
    { // if DRDY is low there are data to read.
     /* Read values from ADC  */
      txbuf[0] = ADS_RDATA_Cmd;
      spiReadWrite(rxbuf, txbuf, 1);    // Send command
      spiReadWrite(rxbuf, nopbuf, 2); // Read result (Writing NOPs)
      *value = (rxbuf[0] << 8) + rxbuf[1]; 
    }
}

/* ---------------------------------------------------------------------------*/
void adsStartSeq(void)
{
  // RDATAC mode is required!!

  adsIrqInit(); //Initialize the IRQ system.
  // Start converting on ch 0.  
  /* Route IDAC current */
  txbuf[0] = ADS_WREG_Cmd | ADS_IDAC1_Reg;
  txbuf[1] = 0; // write 1 byte
  txbuf[2] = ( idacMuxLookup[0] );
  spiReadWrite(rxbuf, txbuf, 3);
  
  // Start ReadDataContinious mode.
  txbuf[0] = ADS_RDATAC_Cmd;
  spiReadWrite(rxbuf, txbuf, 1);

  /* Initially set MUX0 */
  txbuf[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
  txbuf[1] = 0; // write 1 byte
  txbuf[2] = ( (ADS_BCS << 6) | (muxLookup[0]) );
  spiReadWrite(rxbuf, txbuf, 3);
  
  /* SET the START pin */
  GPIO_SetBits(GPIOB, ADS_START_PIN);
}

/* ---------------------------------------------------------------------------*/
void adsContiniueSeq(void)
{
  GPIO_SetBits(GPIOB, ADS_START_PIN);
}

/* ---------------------------------------------------------------------------*/
void adsStoptSeq(void)
{ 
  //De-assert START
  GPIO_ResetBits(GPIOB, ADS_START_PIN);

  // Stop ReadDataContinious mode.
  txbuf[0] = ADS_SDATAC_Cmd;
  spiReadWrite(rxbuf, txbuf, 1);

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
/* Sequential ADC is starte by calling adsStartSeq() which starts the first conversion on ch 0  */
/*                                                                                                                                   */
void ADS_Handler(void)
{
  static uint8_t adcNextCh = 0;
  uint8_t adcCh = 0;
  //use privete rxbuf and txbuf? - As long as "spiReadWrite" is used - no.
  // RDATAC mode is required!!
  // NB! SPI FIFO is 4 bytes deep -> This causes buisy waiting for 2 byts transmit time!!
  // The SPI activities in this function is measured at approx 90 us.

  adcCh = adcNextCh; //current ch is what was next ch
  // Next channel
  if(adcNextCh < 3)
  {
    adcNextCh++;
  }
  else
  {
    adcNextCh = 0;    
  } 

  /* Route IDAC current */
  txbuf[0] = ADS_WREG_Cmd | ADS_IDAC1_Reg;
  txbuf[1] = 0; // write 1 byte
  txbuf[2] = ( idacMuxLookup[adcNextCh] );
  spiReadWrite(rxbuf, txbuf, 3);
  //result of just completed conversion is in rxbuf
  latestConv[adcCh] = (rxbuf[0] << 8) + rxbuf[1]; //
  
  /* Initially set MUX0 */
  txbuf[0] = ADS_WREG_Cmd | ADS_MUX0_Reg;
  txbuf[1] = 0; // write 1 byte
  txbuf[2] = ( (ADS_BCS << 6) | (muxLookup[adcNextCh]) );
  spiReadWrite(rxbuf, txbuf, 3);

  // Next channel
  if(0 == adcNextCh)
  {
    //adsTick(); //ADS time tick
    //If conv is don i bursts og 4 ch: 
    GPIO_ResetBits(GPIOB, ADS_START_PIN);
    //restart is then "GPIO_SetBits(GPIOB, ADS_START_PIN);" right?
  } 
  EXTI_ClearITPendingBit(ADS_EXTI_LINE);
}

