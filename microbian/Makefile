# microbian/Makefile
# Copyright (c) 2020 J. M. Spivey

all: microbian.a startup.o

CPU = -mcpu=cortex-m0plus -mthumb
#CFLAGS = -O -g -Wall -ffreestanding
CFLAGS = -Wall -g -nostartfiles -O -DF_CPU=48000000UL -ffreestanding \
-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0 -DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS \
-DUSE_TINYUSB



CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
AR = arm-none-eabi-ar

#INCLUDE = -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL \
#          -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/samd21 \
#          -I ~/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/Core/Include \
#          -I ./variant-trinketm0

VARIANT_PATH = ../variant-trinketm0
INCLUDE = -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL \
          -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/samd21 \
          -I ~/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/Core/Include \
          -I $(VARIANT_PATH)


#DRIVERS = timer.o serial.o i2c.o radio.o display.o
DRIVERS = timer.o serial.o onboard_dotstar.o

#BODGE = cortex_handlers.o delay.o temp-wiring.o arduino-startup.o polling-uart.o
#moved code from temp-wiring.c and arduino-startup.c into startup.c
#move cortex handler code to end of startup.c

BODGE = polling-uart.o

MICROBIAN = microbian.o mpx-m0.o $(DRIVERS) lib.o $(BODGE)

microbian.a: $(MICROBIAN)
	$(AR) cr $@ $^

%.o: %.c
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.o: %.s
	$(AS) $(CPU) $< -o $@

clean: force
	rm -f microbian.a *.o

force:

###

$(MICROBIAN) startup.o: microbian.h hardware.h lib.h
