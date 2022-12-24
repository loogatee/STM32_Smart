#
#
#
CFLAGS  = -std=c99 -Werror -Wall
IFLAGS  = -I./src -I./Libraries/STM32F10x_StdPeriph_Driver/inc -I./Libraries/CMSIS/Device/ST/STM32F10x/Include -I./Libraries/CMSIS/Include
DFLAGS  = -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D_GNUC_
MFLAGS  = -mthumb -mcpu=cortex-m3 -mlittle-endian
FFLAGS  = -fno-common -ffunction-sections -fdata-sections -fstrict-volatile-bitfields
ASFLAGS = -Wa,--warn -x assembler-with-cpp

WLFLAGS = -Wl,--cref,-u,Reset_Handler -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80 -Wl,--start-group -Wl,--end-group -Wl,--no-warn-rwx-segments -Wl,--nostdlib
LFLAGS  = $(MFLAGS) -T./stm32_flash.ld -static $(WLFLAGS)


GCC = arm-none-eabi-gcc


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

OBJS_REL = $(C_SOURCES:%.c=RELEASE/%.o) $(S_SOURCES:%.s=RELEASE/%.o)
OBJS_DEB = $(C_SOURCES:%.c=DEBUG/%.o)   $(S_SOURCES:%.s=DEBUG/%.o)

#
#	$(GCC) -g -gdwarf-2 $(LFLAGS) -specs=nano.specs -specs=nosys.specs -Wl,-Map=DEBUG/F103shell.map $^ -o $@ -lc
#
#
DEBUG/F103shell.elf: $(OBJS_DEB)
	$(GCC) $(LFLAGS) -specs=nano.specs -specs=nosys.specs -Wl,-Map=DEBUG/F103shell.map $^ -o $@ -lc
	@echo
	arm-none-eabi-objcopy -O binary DEBUG/F103shell.elf DEBUG/F103shell.bin

RELEASE/F103shell.elf: $(OBJS_REL)
	$(GCC) $(LFLAGS) -specs=nano.specs -specs=nosys.specs -Wl,-Map=DEBUG/F103shell.map $^ -o $@ -lc
	@echo
	arm-none-eabi-objcopy -O binary RELEASE/F103shell.elf RELEASE/F103shell.bin


.PHONY:  debug 
.PHONY:  release

debug: DEBUG/F103shell.elf

release: RELEASE/F103shell.elf

clean:
	rm -f DEBUG/*.o DEBUG/*.su DEBUG/*.elf DEBUG/*.map DEBUG/*.list DEBUG/*.bin
	rm -f RELEASE/*.o RELEASE/*.su RELEASE/*.elf RELEASE/*.map RELEASE/*.list RELEASE/*.bin




DEBUG/%.o:   Libraries/STM32F10x_StdPeriph_Driver/src/%.c | DEBUG
	$(GCC) -g -c $< -O0 $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

DEBUG/%.o:   src/%.c | DEBUG
	$(GCC) -g -c $< -O0 $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

DEBUG/%.o:   src/%.s | DEBUG
	$(GCC) -g -c $(MFLAGS) $(DFLAGS) $(IFLAGS) $(ASFLAGS) -o $@ $<
	@echo


RELEASE/%.o:   Libraries/STM32F10x_StdPeriph_Driver/src/%.c | RELEASE
	$(GCC) -c $< -Os $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

RELEASE/%.o:   src/%.c | RELEASE
	$(GCC) -c $< -Os $(OPTIM) $(CFLAGS) $(IFLAGS) $(DFLAGS) $(MFLAGS) $(FFLAGS) -o $@
	@echo

RELEASE/%.o:   src/%.s | RELEASE
	$(GCC) -c $(MFLAGS) $(DFLAGS) $(IFLAGS) $(ASFLAGS) -o $@ $<
	@echo

DEBUG:
	mkdir DEBUG

RELEASE:
	mkdir RELEASE
