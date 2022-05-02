CC = arm-none-eabi-gcc
CPU = -mcpu=cortex-m0plus -mthumb

CFLAGS= -Wall -g -nostartfiles -O -DF_CPU=48000000UL -ffreestanding \
-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0 -DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS -DUSB_VID=0x239A -DUSB_PID=0x801e \
-DUSBCON -DUSB_CONFIG_POWER=100 "-DUSB_MANUFACTURER=\"Adafruit\"" "-DUSB_PRODUCT=\"Trinket M0\"" \
-DUSE_TINYUSB -DARDUINO_ARCH_SAMD

INCLUDE = -I . -I ./common -I ./arduino -I ./common -I ./device -I ./fromcore \
	-I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL \
        -I ~/.arduino15/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/samd21 \
        -I ~/.arduino15/packages/adafruit/tools/CMSIS/5.4.0/CMSIS/Core/Include \
        -I ../../../variant-trinketm0



AR = arm-none-eabi-ar

OBJECTS = ./class/cdc/cdc_device.c.o \
	./class/audio/audio_device.c.o \
	./class/dfu/dfu_rt_device.c.o \
	./class/dfu/dfu_device.c.o \
	./class/hid/hid_device.c.o \
	./class/msc/msc_device.c.o \
	./class/midi/midi_device.c.o \
	./class/net/ecm_rndis_device.c.o \
	./class/net/ncm_device.c.o \
	./class/bth/bth_device.c.o \
	./class/usbtmc/usbtmc_device.c.o \
	./class/vendor/vendor_device.c.o \
	./device/usbd_control.c.o \
	./device/usbd.c.o \
	./common/tusb_fifo.c.o \
	./portable/microchip/samd/dcd_samd.c.o \
	./tusb.c.o \
	./arduino/hid/Adafruit_USBD_HID.cpp.o \
	./arduino/webusb/Adafruit_USBD_WebUSB.cpp.o \
	./arduino/Adafruit_USBD_Device.cpp.o \
	./arduino/Adafruit_USBD_CDC.cpp.o \
	./arduino/msc/Adafruit_USBD_MSC.cpp.o \
	./arduino/midi/Adafruit_USBD_MIDI.cpp.o \
	./arduino/ports/samd/Adafruit_TinyUSB_samd.cpp.o \
	./arduino/Adafruit_TinyUSB_API.cpp.o \
        glue.o

tinyusb.a: $(OBJECTS)
	rm -f tinyusb.a
	$(AR) cr $@ $^

%.o: %.cpp
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.cpp.o: %.cpp
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.c.o: %.c
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

%.o: %.c
	$(CC) $(CPU) $(CFLAGS) $(INCLUDE) -c $< -o $@

