all: kernel.bin

PORT=ttyACM0
BOSSAC=/home/nick/.arduino15/packages/adafruit/tools/bossac/1.8.0-48-gb176eee/bossac

CC = arm-none-eabi-gcc
CPU = -mcpu=cortex-m0plus -mthumb #microbian uses cortex-m0 not plus
#CFLAGS = -O -g -Wall -ffreestanding
#CRELEASEFLAGS = -O -Wall -ffreestanding
#CFLAGS=-g -nostartfiles -O0 -DF_CPU=48000000UL -mcpu=cortex-m0plus -mthumb

#CFLAGS=-g -nostartfiles -O0 -DF_CPU=48000000UL -mcpu=cortex-m0plus -mthumb \
#-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0 -DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS -DUSB_VID=0x239A -DUSB_PID=0x801e \
#-DUSBCON -DUSB_CONFIG_POWER=100 "-DUSB_MANUFACTURER=\"Adafruit\"" "-DUSB_PRODUCT=\"Trinket M0\"" -DUSE_TINYUSB

#remove -DUSE_TINYUSB for now
CFLAGS= -Wall -g -nostartfiles -O -DF_CPU=48000000UL -ffreestanding \
-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0 -DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS -DUSB_VID=0x239A -DUSB_PID=0x801e \
-DUSBCON -DUSB_CONFIG_POWER=100 "-DUSB_MANUFACTURER=\"Adafruit\"" "-DUSB_PRODUCT=\"Trinket M0\""

#LFLAGS=-Wl,-T,trinketm0.ld

#INCLUDE = -I ../microbian
INCLUDE = -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL \
          -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/samd21 \
          -I ~/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/Core/Include \
          -I ./variant-trinketm0

AS = arm-none-eabi-as
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
NM = arm-none-eabi-nm


vpath %.h ./microbian


%.o: %.cpp
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.o: %.c
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.o: %.s
	$(AS) $(CPU) $< -o $@

#%.elf: %.o adc-VDD-only.o ../microbian/microbian.a ../microbian/startup.o
#        $(CC) $(CPU) $(CFLAGS) -T ../microbian/nRF51822.ld \
#                $^ -nostdlib -lgcc -lc -o $@ -Wl,-Map,$*.map
#        $(SIZE) $@

%.elf: cortex_handlers.o startup.o temp-wiring.o hooks.o delay.o Reset.o blink.o force_bootloader.o
	$(CC) $(CPU) $(CFLAGS) -T ./variant-trinketm0/flash_with_bootloader.ld \
		$^ -nostdlib -lgcc -lc -o $@ -Wl,-Map,$*.map
	$(SIZE) $@



#./microbian/microbian.a:
#        $(MAKE) -C $(@D) all

#build:
#	${CC} ${CFLAGS} ${LFLAGS} *.c -o kernel.elf
#	${OBJCOPY} kernel.elf -O binary kernel.bin

disasm:
	${OBJDUMP} -d kernel.elf > kernel.elf.asm
	${NM} kernel.elf > kernel.elf.nm
	hexdump kernel.bin > kernel.bin.hexdump

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

./microbian/microbian.a:
	$(MAKE) -C $(@D) all

# Nuke the default rules for building executables
SORRY = echo "Please say 'make $@.hex' to compile '$@'"
%: %.s; @$(SORRY)
%: %.o; @$(SORRY)


clean:
	rm -f *.bin *.elf *.nm *.map *.o *elf.asm *.hexdump #2> /dev/null


flash: kernel.bin
	${BOSSAC} -i -d --port=${PORT} -i -e -w -v kernel.bin -R --offset 0x2000

# Don't delete intermediate files
.SECONDARY:

###

#battery.o adc-VDD-only.o: microbian.h lib.h hardware.h

