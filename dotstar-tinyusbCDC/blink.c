
/*
 *
 * Physical address from SAMD21E18A datasheet
 *
 * */
//#define WITHOUT_SAM_H
/*
//trinketm0
// GPIO 7 & 8 - DotStar internal data/clock
  // ----------------------------------------
  { PORTA,  0, PIO_DIGITAL, 0, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // ADC/AIN[4]
  { PORTA,  1, PIO_DIGITAL, 0, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // ADC/AIN[5]
//looking like bitbang interface
#define INTERNAL_DS_DATA     (7u)   -> itsym0 41u =real1 DATA so itsym0 swapped relative to trinketm0
#define INTERNAL_DS_CLK      (8u)   -> itsym0 40u =real0 CLK
#define PIN_DOTSTAR_DATA     (7u)
#define PIN_DOTSTAR_CLK      (8u)
Adafruit_DotStar strip = Adafruit_DotStar(NUM_PIXELS, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BGR);
spi_dev = new Adafruit_SPIDevice(-1, clock, -1, data, 8000000);
(cspin=-1; sck=clock, miso=-1, mosi=data, freq=8000000, dataOrder=SPI_BITORDER_MSBFIRST, dataMode = SPI_MODE0)
See ~/Arduino/libraries/Adafruit_BusIO/Adafruit_SPIDevice.*

  // [START FRAME]
  for (i = 0; i < 4; i++)
    spi_dev->transfer(0x00);

  // [PIXEL DATA]
  if (brightness) {            // Scale pixel brightness on output
    do {                       // For each pixel...
      spi_dev->transfer(0xFF); //  Pixel start
      for (i = 0; i < 3; i++)
        spi_dev->transfer((*ptr++ * b16) >> 8); // Scale, write
    } while (--n);
  } else {                     // Full brightness (no scaling)
    do {                       // For each pixel...
      spi_dev->transfer(0xFF); //  Pixel start
      for (i = 0; i < 3; i++)
        spi_dev->transfer(*ptr++); // R,G,B
    } while (--n);
  }

  // [END FRAME]
  // Four end-frame bytes are seemingly indistinguishable from a white
  // pixel, and empirical testing suggests it can be left out...but it's
  // always a good idea to follow the datasheet, in case future hardware
  // revisions are more strict (e.g. might mandate use of end-frame
  // before start-frame marker). i.e. let's not remove this. But after
  // testing a bit more the suggestion is to use at least (numLeds+1)/2
  // high values (1) or (numLeds+15)/16 full bytes as EndFrame. For details
  // see also:
  // https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superle$
  for (i = 0; i < ((numLEDs + 15) / 16); i++)
    spi_dev->transfer(0xFF);


*/

//#define WITHOUT_SAM_H

#include "onboard_dotstar.h"
#ifndef WITHOUT_SAM_H
#include "sam.h"
#endif
#include "delay.h"
#include "mcu_pins.h"
#include "microbian.h"
//#include "./microbian/microbian.h"
//#include "lib.h"

#ifdef WITHOUT_SAM_H
#define GPIO_BASE 0x41004400
#define PORTDIRSET 0x41004408
#define PORTOUT 0x41004410
#define PORTCLR 0x41004414
#define LED_GPIO_BIT PIN_REAL_LED
//#define DOTSTAR_GPIO_CLK_BIT PIN_REAL_DOTSTAR_CLK
//#define DOTSTAR_GPIO_DAT_BIT PIN_REAL_DOTSTAR_DAT
#endif

static int USB_TASK;
#define USBPUTC 334
#define USBGETC 335
void USBSerial_putc(char ch);
int USBSerial_getc(void);


static int A_TASK;
static int B_TASK;
#define PINGPONG 333


extern void force_bootloader(void);
int count = 200*4;
#ifndef WITHOUT_SAM_H
uint32_t port = 0;
const uint32_t pin = PIN_REAL_LED;
const uint32_t pinMask = (1ul << pin);
#endif

volatile int c;
//volatile char c;

void test_taskA(int arg)
{
//   int client;
   message m;
   int i=0;
   while (1) {
	receive(PINGPONG, &m);
//        client = m.sender;
        int idx=i/2;
        i++;
        if (idx>6) i=0;

        dotstar_show(idx); //called from here get white pixel
        Uart_write('X');
//        c = USBSerial_getc();
//	USBSerial_putc('U');
	/* Clear LED output pin*/
#ifdef WITHOUT_SAM_H
	*(unsigned int*)PORTCLR = (1 << LED_GPIO_BIT);
#else
        PORT->Group[port].OUTCLR.reg = pinMask;
#endif
   }
}

void test_taskB(int arg)
{
//   int client;
   message m;
   while (1) {
	receive(PINGPONG, &m);
//        client = m.sender;
        //dotstar_show(DOT_BLACK);
        dotstar_show(DOT_BLACK);
        Uart_write('Y');
//        if (c==-1)
//		USBSerial_putc('.');
//	else
//		USBSerial_putc(c);


		/* Set LED output pin*/
#ifdef WITHOUT_SAM_H
	*(unsigned int*)PORTOUT = (1 << LED_GPIO_BIT);
#else
        PORT->Group[port].OUTSET.reg = pinMask;
#endif

   }
}

void test_taskT(int arg)
{
   message m;
   timer_pulse(250); //pulse 50 worked for ttyACM0, 100 also
   while (1) {
	receive(PING, NULL);
        count--;

        if (count <= 1) //10ms * 10000 = 100secs ish, check this works before adding to microbian port for first t$
        {
           Uart_end();
           force_bootloader();
        }
        if ((count & 0x3) == 0x0)
            send(A_TASK, PINGPONG, &m);
        else if ((count & 0x3) == 0x2)
            send(B_TASK, PINGPONG, &m);

   }
}

void test_taskUSB(int arg)
{
   message m;
   int client;
   int reader = -1;
   char ch;

extern void __libc_init_array(void);

   __libc_init_array();
   TinyUSB_Device_Init(0);
   delayMicroseconds(1000);

   USBSerial_begin(115200);

   timer_pulse(100); //pulse 50 worked for ttyACM0, 100 also
   while (1) {
	receive(ANY, &m);
	client = m.sender;
//m.int1;
	switch (m.type) {
	case PING:
	   TinyUSB_Device_Task();
	   TinyUSB_Device_FlushCDC();
	   break;
	case USBGETC:
//	   if (reader >= 0)
//		panic("Two clients cannot wait for input at once");
	   reader = client;
	   m.int1 = USBSerial_read(); //will return -1 if none available
	   send(reader, REPLY, &m);
	   reader = -1;
	   break;
	case USBPUTC:
            ch = m.int1;
            if (ch == '\n') USBSerial_write('\r');
            USBSerial_write(ch);
	   break;
	default:
	   badmesg(m.type);
	}
   }

}

void echo_task(int arg)
{
(void)arg;

	while (1) {
	   timer_delay(100);
           c = USBSerial_getc();
//	   USBSerial_putc('U');
           if (c==-1)
	   {
		USBSerial_putc('.');
	   	USBSerial_putc('\r');
	   }
	   else
	   {
		if (c == '\r')
		   USBSerial_putc('\n');
		else
		   USBSerial_putc(c);

	   }
	}

}

/* serial_putc -- queue a character for output */
void USBSerial_putc(char ch)
{
    message m;
    m.int1 = ch;
    send(USB_TASK, USBPUTC, &m);
}

/* serial_getc -- request an input character */
int USBSerial_getc(void)
{
    message m;
    send(USB_TASK, USBGETC, NULL);
    receive(REPLY, &m);
    return m.int1;
}


//see WVariant.h EPortType port = PORTA;
void init() {
	timer_init();
        dotstar_init();
//        delayMicroseconds(5);
        //dotstar_show();
//        dotstar_show_yellow();

        //dotstar_show(); //using 2uS period for first go, calculates as 0
        //arduino_init(); called from __reset() now


        Uart_begin(9600);
	/* Set output direction for LED pin*/
#ifdef WITHOUT_SAM_H
	* (unsigned int*)PORTDIRSET |= (1 << LED_GPIO_BIT);
#else
      // enable input, to support reading back values, with pullups disabled
      PORT->Group[port].PINCFG[pin].reg = (uint8_t) (PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR);
      // Set pin to output mode
      PORT->Group[port].DIRSET.reg = pinMask;
#endif
  A_TASK = start("TestA", test_taskA, 0, STACK);
  B_TASK = start("TestB", test_taskB, 0, STACK);

           start("TestTimer", test_taskT, 0, STACK);
  USB_TASK = start("USBCDC", test_taskUSB, 0, STACK);
  	start("echo", echo_task, 0, 256);

}
