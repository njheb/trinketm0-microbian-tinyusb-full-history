#include "Adafruit_TinyUSB.h"

Adafruit_USBD_CDC Serial;

extern "C"{
void USBSerial_begin(int baud)
{
	Serial.begin(baud);
}

void USBSerial_write(int c)
{
	if (Serial)
		Serial.write(c);
}

int USBSerial_read(void)
{
	if (Serial)
		return Serial.read();
}
