#
#
#
CFLAGS  = -std=c99 -Wall -specs=nano.specs
IFLAGS  = -I./src -I./Libraries/STM32F10x_StdPeriph_Driver/inc -I./Libraries/CMSIS/Device/ST/STM32F10x/Include -I./Libraries/CMSIS/Include
DFLAGS  = -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D_GNUC_
MFLAGS  = -mthumb -mcpu=cortex-m3 -mlittle-endian
FFLAGS  = -fno-common -ffunction-sections -fdata-sections -fstrict-volatile-bitfields
ASFLAGS = -Wa,--warn -x assembler-with-cpp -specs=nano.specs

WLFLAGS = -Wl,-cref,-u,Reset_Handler -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 -Wl,--start-group -Wl,--end-group -specs=nano.specs
LFLAGS  = -T./stm32_flash.ld -specs=nosys.specs -static


GCC         = arm-atollic-eabi-gcc
ATOLLIC_DIR = /opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.3.0

TARGET  = F103shell
DEBUG   = Debug
RELEASE = Release


S_SOURCES = startup_stm32f10x_md.s

C_SOURCES = system_stm32f10x.c     \
            stm32f1xx_it.c         \
            main.c                 \
            Cmds.c                 \
            Ticker.c               \
            Uart_In.c              \
            Uart_Out.c             \
            Utils.c                \
            misc.c                 \
            stm32f10x_adc.c        \
            stm32f10x_bkp.c        \
            stm32f10x_can.c        \
            stm32f10x_cec.c        \
            stm32f10x_crc.c        \
            stm32f10x_dac.c        \
            stm32f10x_dbgmcu.c     \
            stm32f10x_dma.c        \
            stm32f10x_exti.c       \
            stm32f10x_flash.c      \
            stm32f10x_fsmc.c       \
            stm32f10x_gpio.c       \
            stm32f10x_i2c.c        \
            stm32f10x_iwdg.c       \
            stm32f10x_pwr.c        \
            stm32f10x_rcc.c        \
            stm32f10x_rtc.c        \
            stm32f10x_sdio.c       \
            stm32f10x_spi.c        \
            stm32f10x_tim.c        \
            stm32f10x_usart.c      \
            stm32f10x_wwdg.c

OBJS_REL = $(C_SOURCES:%.c=$(RELEASE)/%.o) $(S_SOURCES:%.s=$(RELEASE)/%.o)
OBJS_DEB = $(C_SOURCES:%.c=$(DEBUG)/%.o)   $(S_SOURCES:%.s=$(DEBUG)/%.o)



$(DEBUG)/$(TARGET).elf: $(OBJS_DEB)
	$(GCC) -o $@ $^ $(MFLAGS) $(LFLAGS) $(WLFLAGS) -Wl,-Map=$(DEBUG)/$(TARGET).map
	@echo
	$(ATOLLIC_DIR)/ide/jre/bin/java -jar $(ATOLLIC_DIR)/Tools/arm-atollic-reports.jar sizeinfo list $(DEBUG)/$(TARGET).elf
	@echo
	arm-atollic-eabi-objcopy -O binary $(DEBUG)/$(TARGET).elf $(DEBUG)/$(TARGET).bin


$(RELEASE)/$(TARGET).elf: $(OBJS_REL)
	$(GCC) -o $@ $^ $(MFLAGS) $(LFLAGS) $(WLFLAGS) -Wl,-Map=$(RELEASE)/$(TARGET).map
	@echo
	$(ATOLLIC_DIR)/ide/jre/bin/java -jar $(ATOLLIC_DIR)/Tools/arm-atollic-reports.jar sizeinfo list $(RELEASE)/$(TARGET).elf
	@echo
	arm-atollic-eabi-objcopy -O binary $(RELEASE)/$(TARGET).elf $(RELEASE)/$(TARGET).bin


.PHONY:  debug 
.PHONY:  release

debug: $(DEBUG)/F103shell.elf

release: $(RELEASE)/F103shell.elf

clean:
	rm -f $(DEBUG)/*
	rm -f $(RELEASE)/*




$(DEBUG)/%.o:   Libraries/STM32F10x_StdPeriph_Driver/src/%.c | $(DEBUG)
	$(GCC) -c $< -O0 -g $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

$(DEBUG)/%.o:   src/%.c | $(DEBUG)
	$(GCC) -c $< -O0 -g $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

$(DEBUG)/%.o:   src/%.s | $(DEBUG)
	$(GCC) -c -g $(MFLAGS) $(DFLAGS) $(IFLAGS) $(ASFLAGS) -o $@ $<
	@echo


$(RELEASE)/%.o:   Libraries/STM32F10x_StdPeriph_Driver/src/%.c | $(RELEASE)
	$(GCC) -c $< -Os $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

$(RELEASE)/%.o:   src/%.c | $(RELEASE)
	$(GCC) -c $< -Os $(OPTIM) $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

$(RELEASE)/%.o:   src/%.s | $(RELEASE)
	$(GCC) -c $(MFLAGS) $(DFLAGS) $(IFLAGS) $(ASFLAGS) -o $@ $<
	@echo



$(DEBUG):
	mkdir $(DEBUG)

$(RELEASE):
	mkdir $(RELEASE)


