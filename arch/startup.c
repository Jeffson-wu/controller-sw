typedef void isr_handler(void);

extern unsigned long _estack;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _sidata;
extern unsigned long _sbss;
extern unsigned long _ebss;

extern int main(int argc, char **argv);

void Reset_Handler(void);
void NMI_Handler(void) __attribute__ ((weak));
void HardFault_Handler(void) __attribute__ ((weak));
void MemManage_Handler(void) __attribute__ ((weak));
void BusFault_Handler(void) __attribute__ ((weak));
void UsageFault_Handler(void) __attribute__ ((weak));
void SVC_Handler(void) __attribute__ ((weak));
void DebugMon_Handler(void) __attribute__ ((weak));
void PendSV_Handler(void) __attribute__ ((weak));
void SysTick_Handler(void) __attribute__ ((weak));
void Default_Handler(void) __attribute__ ((weak));
void UART2_TX_Handler(void) __attribute__ ((weak));
void UART2_RX_Handler(void) __attribute__ ((weak));

/* Vector table for (cl) Connectivity Line devices (STM32F105xx and STM32F107xx) */
isr_handler *vectors[] __attribute__ ((section(".isr_vector"))) = {
    (isr_handler *)&_estack,    /* Initial SP */
    Reset_Handler,              /* Reset */
    NMI_Handler,                /* NMI */
    HardFault_Handler,          /* HardFault */
    MemManage_Handler,          /* MemManage */
    BusFault_Handler,           /* BusFault */
    UsageFault_Handler,         /* UsageFault */
    0,
    0,
    0,
    0,
    SVC_Handler,                /* SVCall */
    DebugMon_Handler,           /* DebugMon */
    0,
    PendSV_Handler,             /* PendSV */
    SysTick_Handler,            /* SysTick */
    Default_Handler,            /* 0: WWDG */
    Default_Handler,            /* 1: PVD */
    Default_Handler,            /* 2: TAMPER */
    Default_Handler,            /* 3: RTC */
    Default_Handler,            /* 4: FLASH */
    Default_Handler,            /* 5: RCC */
    Default_Handler,            /* 6: EXTI0 */
    Default_Handler,            /* 7: EXTI1 */
    Default_Handler,            /* 8: EXTI2 */
    Default_Handler,            /* 9: EXTI3 */
    Default_Handler,            /* 10: EXTI4 */
    Default_Handler,            /* 11: DMA1_Channel1 */
    Default_Handler,            /* 12: DMA1_Channel2 */
    Default_Handler,            /* 13: DMA1_Channel3 */
    Default_Handler,            /* 14: DMA1_Channel4 */
    Default_Handler,            /* 15: DMA1_Channel5 */
    UART2_RX_Handler,           /* 16: DMA1_Channel6 */
    UART2_TX_Handler,           /* 17: DMA1_Channel7 */
    Default_Handler,            /* 18: ADC1_2 */
    Default_Handler,            /* 19: CAN1_TX */
    Default_Handler,            /* 20: CAN1_RX0 */
    Default_Handler,            /* 21: CAN1_RX1 */
    Default_Handler,            /* 22: CAN1_SCE */
    Default_Handler,            /* 23: EXTI9_5 */
    Default_Handler,            /* 24: TIM1_BRK */
    Default_Handler,            /* 25: TIM1_UP */
    Default_Handler,            /* 26: TIM1_TRG_COM */
    Default_Handler,            /* 27: TIM1_CC */
    Default_Handler,            /* 28: TIM2 */
    Default_Handler,            /* 29: TIM3 */
    Default_Handler,            /* 30: TIM4 */
    Default_Handler,            /* 31: I2C1_EV */
    Default_Handler,            /* 32: I2C1_ER */
    Default_Handler,            /* 33: I2C2_EV */
    Default_Handler,            /* 34: I2C2_ER */
    Default_Handler,            /* 35: SPI1 */
    Default_Handler,            /* 36: SPI2 */
    Default_Handler,            /* 37: USART1 */
    Default_Handler,            /* 38: USART2 */
    Default_Handler,            /* 39: USART3 */
    Default_Handler,            /* 40: EXTI15_10 */
    Default_Handler,            /* 41: RTC_Alarm */
    Default_Handler,            /* 42: OTG_FS_WKUP */
    0,                          /* 43: reserved */
    0,                          /* 44: reserved */
    0,                          /* 45: reserved */
    0,                          /* 46: reserved */
    0,                          /* 47: reserved */
    0,                          /* 48: reserved */
    0,                          /* 49: reserved */
    Default_Handler,            /* 50: TIM5 */
    Default_Handler,            /* 51: SPI3 */
    Default_Handler,            /* 52: UART4 */
    Default_Handler,            /* 53: UART5 */
    Default_Handler,            /* 54: TIM6 */
    Default_Handler,            /* 55: TIM7 */
    Default_Handler,            /* 56: DMA2_Channel1 */
    Default_Handler,            /* 57: DMA2_Channel2 */
    Default_Handler,            /* 58: DMA2_Channel3 */
    Default_Handler,            /* 59: DMA2_Channel4 */
    Default_Handler,            /* 60: DMA2_Channel5 */
    Default_Handler,            /* 61: ETH */
    Default_Handler,            /* 62: ETH_WKUP */
    Default_Handler,            /* 63: CAN2_TX */
    Default_Handler,            /* 64: CAN2_RX0 */
    Default_Handler,            /* 65: CAN2_RX1 */
    Default_Handler,            /* 66: CAN2_SCE */
    Default_Handler,            /* 67: OTG_FS */
};

void endless_loop(void)
{
    while (1) {};
}

void NMI_Handler(void)
{
    endless_loop();
}

void HardFault_Handler(void)
{
   endless_loop();
}

void MemManage_Handler(void)
{
   endless_loop();
}

void BusFault_Handler(void)
{
   endless_loop();
}

void UsageFault_Handler(void)
{
   endless_loop();
}

void SVC_Handler(void)
{
    endless_loop();
}

void DebugMon_Handler(void)
{
    endless_loop();
}

void PendSV_Handler(void)
{
    endless_loop();
}

void SysTick_Handler(void)
{
    endless_loop();
}

void Default_Handler(void)
{
    endless_loop();
}

void Reset_Handler(void)
{
    int result;
    unsigned long *src, *dst;

    /* Initialize RW memory */
    for (src = &_sidata, dst = &_sdata; dst < &_edata; dst++, src++) {
        *dst = *src;
    }

    /* Initialize BSS memory */
    for (dst = &_sbss; dst < &_ebss; dst++) {
        *dst = 0x0;
    }

    SystemInit();

    __libc_init_array();
    
    result = main(0, 0);
    
    endless_loop();
}

void _init(void)
{
}

void _fini(void)
{
}
