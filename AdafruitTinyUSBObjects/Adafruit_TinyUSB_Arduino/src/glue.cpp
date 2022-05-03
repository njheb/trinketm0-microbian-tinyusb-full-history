#include "Adafruit_TinyUSB.h"


extern Adafruit_USBD_CDC Serial;

extern "C" {

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
int n=-1;
	if (Serial)
		n = Serial.read();
	return n;
}

void YIELD(void){
  TinyUSB_Device_Task();
  TinyUSB_Device_FlushCDC();
}


}
