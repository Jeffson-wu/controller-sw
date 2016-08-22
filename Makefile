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
CXX = $(TARGET)g++
OBJCOPY = $(TARGET)objcopy
ifeg ($(HEATER_INCDIR),)
  INCLUDES = -I ./include -I ./board -I ./arch -I ./freertos/include -I ./ -I ./freertos/portable/GCC/ARM_CM3 -I ./GenericRecorderLibSrc/Include -I./GenericRecorderLibSrc/ConfigurationTemplate -I$(HEATER_INCDIR)
else
  INCLUDES = -I ./include -I ./board -I ./arch -I ./freertos/include -I ./ -I ./freertos/portable/GCC/ARM_CM3 -I ./GenericRecorderLibSrc/Include -I./GenericRecorderLibSrc/ConfigurationTemplate
endif
ifeq ($(TARGET),arm-none-eabi-)
  CFLAGS = -g -O0 -c -Wall -Werror -mcpu=cortex-m3 -mthumb -ffunction-sections -D__START=main -D__STARTUP_CLEAR_BSS -DSTM32F1XX -DUSE_STDPERIPH_DRIVER $(INCLUDES)
else
  CFLAGS = -g -O0 -c -Wall -Werror -D__START=main -D__STARTUP_CLEAR_BSS -DSTM32F1XX -DUSE_STDPERIPH_DRIVER $(INCLUDES) --coverage
  CXXFLAGS = -g -O0 -c -Wall -Werror `pkg-config cpputest --cflags` --coverage
endif

LDFLAGS = -T arch/stm32f1x.ld -mcpu=cortex-m3 -mthumb -nostartfiles -Wl,--gc-section
LIBS = -lc -lgcc -lnosys -lm
OBJECTS_DIR = obj
DEPENDS_DIR = dep
PROGRAM = controller

SOURCES = \
    main.c \
    version.c \
    util.c \
    serial.c \
    modbus.c \
    gdi.c \
    pwm.c \
    adc.c \
    sequencer.c \
    cooleandlidtask.c \
    fan.c\
    lidheater.c\
    peltier.c\
    logtask.c \
    pid.c \
    nvs.c \
    debug.c \
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
    freertos/event_groups.c \
    freertos/tasks.c \
    freertos/timers.c \
    freertos/portable/GCC/ARM_CM3/port.c \
    freertos/portable/MemMang/heap_4.c \
    GenericRecorderLibSrc/trcBase.c \
    GenericRecorderLibSrc/trcHardwarePort.c \
    GenericRecorderLibSrc/trcKernel.c \
    GenericRecorderLibSrc/trcKernelPort.c \
    GenericRecorderLibSrc/trcUser.c
    
OBJS = $(patsubst %,$(OBJECTS_DIR)/%,$(SOURCES:%.c=%.o))
OBJECTS = $(OBJS:%.S=%.o)
DEPS = $(patsubst %,$(DEPENDS_DIR)/%,$(SOURCES:%.c=%.d))
DEPENDS = $(DEPS:%.S=%.d)

all: $(SOURCES) $(PROGRAM) $(PROGRAM).hex

.PHONY: version.c

version.c:
	echo '#include "version.h"' > $@ 
	echo '/* This file is AUTO GENERATED - Do NOT Check in!!! */' >> $@
	echo 'char buildRevStr[] = SW_VERSION ;' >> $@ 
	echo -n 'char buildDateStr[] = ' >> $@ 
	date +'"%Y.%m.%d-%T";' >> $@ 
	echo -n "char gitCommitIdStr[] = \"$(shell git describe --always --dirty)\";" >> $@

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

$(OBJECTS_DIR)/%.o: %.cpp
	@echo "Compiling $<"
	@mkdir -p $(OBJECTS_DIR)/$(*D)
	@mkdir -p $(DEPENDS_DIR)/$(*D)
	@$(CXX) $(CXXFLAGS) -Wp,-MD,$(DEPENDS_DIR)/$*.d -o $@ $<
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


TESTS = util_tests

tests_LIBS = -lCppUTest -lCppUTestExt -lgcov --coverage
tests_LDFLAGS = -Wl,--gc-section `pkg-config cpputest --libs` --coverage

util_tests_SOURCES = \
	util.c \
	tests/util_tests.cpp

util_tests_OBJS_temp = $(patsubst %,$(OBJECTS_DIR)/%,$(util_tests_SOURCES:%.c=%.o))
util_tests_OBJS = $(util_tests_OBJS_temp:%.cpp=%.o)
util_tests_OBJECTS = $(util_tests_OBJS:%.S=%.o)
util_tests_DEPS_temp = $(patsubst %,$(DEPENDS_DIR)/%,$(util_tests_SOURCES:%.c=%.d))
util_tests_DEPS = $(util_tests_DEPS_temp:%.cpp=%.d)
util_tests_DEPENDS = $(util_tests_DEPS:%.S=%.d)

util_tests: $(util_tests_OBJECTS)
	@echo -n "Linking $@... "
	@$(CXX) $(tests_LDFLAGS) $(util_tests_OBJECTS) $(tests_LIBS) -o $@
	@echo "done"

-include $(DEPENDS)
-include $(util_tests_DEPENDS)

check: $(TESTS)
	@echo -n "Running CppUTest test cases... "
	./util_tests
	@echo "done"

check_coverage:
	make check
	@echo -n "Generating gcov files... "
	@for file in `find obj -name *.gcno`; do \
		gcov $(util_tests_SOURCES) -p -l -o $$file 1>>gcov_output.txt 2>>gcov_error.txt; \
	done
	@echo "done"
	@rm -rf test_output.txt gcov_output.txt gcov_error.txt

clean:
	@rm -rf $(OBJECTS) $(PROGRAM) $(UPDATEIMAGE) version.c
	@rm -rf $(util_tests_OBJECTS)
	@rm -rf util_tests
	@rm -rf cpputest_*.xml
	@rm -rf *.gcno *.gcda *.gcov

clean-all: clean
	@rm -rf $(DEPENDS)
	@rm -rf $(util_tests_DEPENDS)
