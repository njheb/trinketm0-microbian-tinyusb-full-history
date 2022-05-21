all: ../AdafruitTinyUSBObjects/Adafruit_TinyUSB_Arduino/src/tinyusb.a kernel.bin 

PORT=ttyACM0
BOSSAC=/home/nick/.arduino15/packages/adafruit/tools/bossac/1.8.0-48-gb176eee/bossac

CC = arm-none-eabi-gcc
CPP = arm-none-eabi-g++
CPU = -mcpu=cortex-m0plus -mthumb #microbian uses cortex-m0 not plus
#CFLAGS = -O -g -Wall -ffreestanding
#CRELEASEFLAGS = -O -Wall -ffreestanding
#CFLAGS=-g -nostartfiles -O0 -DF_CPU=48000000UL -mcpu=cortex-m0plus -mthumb

#CFLAGS=-g -nostartfiles -O0 -DF_CPU=48000000UL -mcpu=cortex-m0plus -mthumb \
#-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0 -DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS -DUSB_VID=0x239A -DUSB_PID=0x801e \
#-DUSBCON -DUSB_CONFIG_POWER=100 "-DUSB_MANUFACTURER=\"Adafruit\"" "-DUSB_PRODUCT=\"Trinket M0\"" -DUSE_TINYUSB

#remove -DUSE_TINYUSB for now
#CFLAGS= -Wall -g -nostartfiles -O -DF_CPU=48000000UL -ffreestanding \
#-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0 -DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS -DUSB_VID=0x239A -DUSB_PID=0x801e \
#-DUSBCON -DUSB_CONFIG_POWER=100 "-DUSB_MANUFACTURER=\"Adafruit\"" "-DUSB_PRODUCT=\"Trinket M0\"" \
#-DUSE_TINYUSB
CFLAGS= -Wall -g -O -DF_CPU=48000000UL  \
-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0 -DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS -DUSB_VID=0x239A -DUSB_PID=0x801e \
-DUSBCON -DUSB_CONFIG_POWER=100 "-DUSB_MANUFACTURER=\"Adafruit\"" "-DUSB_PRODUCT=\"Trinket M0\"" \
-DUSE_TINYUSB

#LFLAGS=-Wl,-T,trinketm0.ld

#INCLUDE = -I ../microbian
VARIANT_PATH = ../variant-trinketm0

INCLUDE = -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL \
          -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/samd21 \
          -I ~/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/Core/Include \
          -I $(VARIANT_PATH) \
          -I ../microbian

AS = arm-none-eabi-as
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
NM = arm-none-eabi-nm


vpath %.h ../microbian


%.o: %.cpp
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.o: %.c
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.o: %.s
	$(AS) $(CPU) $< -o $@

APP = blink.o force_bootloader.o

WORKAROUND = ../microbian/startup.o ../microbian/microbian.a ../AdafruitTinyUSBObjects/Adafruit_TinyUSB_Arduino/src/tinyusb.a

#LINK_FILE = microbian_flash_with_bootloader.ld
LINK_FILE = tinyusb-trinket_m0.ld

#note cortex_handlers.c temp-wiring.c arduino-startup.c merged into startup.c
#less good for obviously showing copyright but all comments left intact inside startup.c
#%.elf: $(APP) $(WORKAROUND)
#	$(CC) $(CPU) $(CFLAGS) -T $(VARIANT_PATH)/$(LINK_FILE) \
#		$^ -nostdlib -lgcc -lc -o $@ -Wl,-Map,$*.map
#	$(SIZE) $@
%.elf: $(APP) $(WORKAROUND)
	$(CPP) $(CPU) $(CFLAGS) -T $(VARIANT_PATH)/$(LINK_FILE) \
		$^ -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -lstdc++ -lc -lc -o $@ -Wl,-Map,$*.map -Wl,--start-group -L~/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/Lib/GCC/ -Wl,--end-group
	$(SIZE) $@

disasm:
	${OBJDUMP} -d kernel.elf > kernel.elf.asm
	${NM} kernel.elf > kernel.elf.nm
	hexdump kernel.bin > kernel.bin.hexdump

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

../microbian/microbian.a:
	$(MAKE) -C $(@D) all

../AdafruitTinyUSBObjects/Adafruit_TinyUSB_Arduino/src/tinyusb.a:
	$(MAKE) -C $(@D) all

# Nuke the default rules for building executables
SORRY = echo "Please say 'make $@.hex' to compile '$@'"
%: %.s; @$(SORRY)
%: %.o; @$(SORRY)


clean:
	rm -f *.bin *.elf *.nm *.map *.o *elf.asm *.hexdump #2> /dev/null

ultraclean: clean
	rm -f ../microbian/*.a ../microbian/*.o
	rm -f ../AdafruitTinyUSBObjects/Adafruit_TinyUSB_Arduino/src/tinyusb.a 
	$(MAKE) -C ../AdafruitTinyUSBObjects/Adafruit_TinyUSB_Arduino/src/ clean

flash: kernel.bin
	${BOSSAC} -i -d --port=${PORT} -i -e -w -v kernel.bin -R --offset 0x2000

# Don't delete intermediate files
.SECONDARY:

###

blink.o force_bootloader.o: microbian.h lib.h hardware.h
