# STM32F32103 Project Makefile
#
# Copyright (c) 2013 XTel ApS
#
# Author:
#   Thomas Lykkeberg <tly@xtel.dk>
# Comment:
#   This file forms the basis of any project based
#   on the STM32F1 chip series from STMicroelectronics
#

TARGET = arm-none-eabi-
CC = $(TARGET)gcc
OBJCOPY = $(TARGET)objcopy
INCLUDES = -I ./include -I ./board -I ./arch -I ./freertos/include -I ./ -I ./freertos/portable/GCC/ARM_CM3 -I ./GenericRecorderLibSrc/Include -I./GenericRecorderLibSrc/KernelPorts/FreeRTOS -I./GenericRecorderLibSrc/ConfigurationTemplate
CFLAGS = -g -O0 -c -Wall -mcpu=cortex-m3 -mthumb -D__START=main -D__STARTUP_CLEAR_BSS -DSTM32F1XX -DUSE_STDPERIPH_DRIVER $(INCLUDES)

LDFLAGS = -T arch/stm32f1x.ld -mcpu=cortex-m3 -mthumb -nostartfiles -Wl,--gc-section
LIBS = -lc -lgcc -lnosys
OBJECTS_DIR = obj
DEPENDS_DIR = dep
PROGRAM = controller

SOURCES = \
    main.c \
    util.c \
    serial.c \
    modbus.c \
    gdi.c \
    pwm.c \
    ads1148.c \
    sequencer.c \
    jsmn/decode_script.c \
    jsmn/json.c \
    jsmn/jsmn.c \
    cooleandlidtask.c \
    logtask.c \
	pid.c \
    arch/startup.S \
    arch/system_stm32f10x.c \
    arch/core_cm3.c \
    board/stm3210c-eval.c \
    drivers/misc.c \
    drivers/stm32f10x_adc.c \
    drivers/stm32f10x_bkp.c \
    drivers/stm32f10x_can.c \
    drivers/stm32f10x_cec.c \
    drivers/stm32f10x_crc.c \
    drivers/stm32f10x_dac.c \
    drivers/stm32f10x_dbgmcu.c \
    drivers/stm32f10x_dma.c \
    drivers/stm32f10x_exti.c \
    drivers/stm32f10x_flash.c \
    drivers/stm32f10x_fsmc.c \
    drivers/stm32f10x_gpio.c \
    drivers/stm32f10x_i2c.c \
    drivers/stm32f10x_iwdg.c \
    drivers/stm32f10x_pwr.c \
    drivers/stm32f10x_rcc.c \
    drivers/stm32f10x_rtc.c \
    drivers/stm32f10x_sdio.c \
    drivers/stm32f10x_spi.c \
    drivers/stm32f10x_tim.c \
    drivers/stm32f10x_usart.c \
    drivers/stm32f10x_wwdg.c \
    freertos/croutine.c \
    freertos/list.c \
    freertos/queue.c \
    freertos/tasks.c \
    freertos/timers.c \
    freertos/portable/GCC/ARM_CM3/port.c \
    freertos/portable/MemMang/heap_4.c \
    GenericRecorderLibSrc/trcBase.c \
    GenericRecorderLibSrc/trcHardwarePort.c \
    GenericRecorderLibSrc/trcKernel.c \
    GenericRecorderLibSrc/trcUser.c \
    GenericRecorderLibSrc/KernelPorts/FreeRTOS/trcKernelPort.c
    
OBJS = $(patsubst %,$(OBJECTS_DIR)/%,$(SOURCES:%.c=%.o))
OBJECTS = $(OBJS:%.S=%.o)
DEPS = $(patsubst %,$(DEPENDS_DIR)/%,$(SOURCES:%.c=%.d))
DEPENDS = $(DEPS:%.S=%.d)

all: $(SOURCES) $(PROGRAM) $(PROGRAM).hex

$(PROGRAM): $(OBJECTS)
	@echo -n "Linking $@... "
	@$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@
	@echo "done"

$(PROGRAM).hex: $(PROGRAM)
	@$(OBJCOPY) -O ihex $< $@

$(OBJECTS_DIR)/%.o: %.c
	@echo "Compiling $<"
	@mkdir -p $(OBJECTS_DIR)/$(*D)
	@mkdir -p $(DEPENDS_DIR)/$(*D)
	@$(CC) $(CFLAGS) -Wp,-MD,$(DEPENDS_DIR)/$*.d -o $@ $<
	@sed -e 's,^$*\.o,$@,' < $(DEPENDS_DIR)/$*.d > $(DEPENDS_DIR)/$*.P
	@rm $(DEPENDS_DIR)/$*.d
	@mv $(DEPENDS_DIR)/$*.P $(DEPENDS_DIR)/$*.d

$(OBJECTS_DIR)/%.o: %.S
	@echo "Assembling $<"
	@mkdir -p $(OBJECTS_DIR)/$(*D)
	@mkdir -p $(DEPENDS_DIR)/$(*D)
	@$(CC) $(CFLAGS) -Wp,-MD,$(DEPENDS_DIR)/$*.d -o $@ $<
	@sed -e 's,^$*\.o,$@,' < $(DEPENDS_DIR)/$*.d > $(DEPENDS_DIR)/$*.P
	@rm $(DEPENDS_DIR)/$*.d
	@mv $(DEPENDS_DIR)/$*.P $(DEPENDS_DIR)/$*.d

-include $(DEPENDS)

clean:
	@rm -rf $(OBJECTS) $(PROGRAM) $(UPDATEIMAGE)

clean-all: clean
	@rm -rf $(DEPENDS)
