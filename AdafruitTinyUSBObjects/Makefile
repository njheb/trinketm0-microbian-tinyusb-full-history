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
	./arduino/Adafruit_TinyUSB_API.cpp.o

tinyusb.a: $(OBJECTS)
	rm -f tinyusb.a
	$(AR) cr $@ $^
