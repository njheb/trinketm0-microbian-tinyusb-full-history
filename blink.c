
/*
 *
 * Physical address from SAMD21E18A datasheet
 *
 * */
#include "sam.h"
//#define IMPROPER
#ifdef IMPROPER
#define GPIO_BASE 0x41004400
#define PORTDIRSET 0x41004408
#define PORTOUT 0x41004410
#define PORTCLR 0x41004414
#define LED_GPIO_BIT 10
#endif


/** GPIO Register set */
volatile unsigned int* gpio;

/** Simple loop variable */
volatile unsigned int tim;

extern void force_bootloader(void);
int count = 100;
//see WVariant.h EPortType port = PORTA;
/** Main function - we'll never return from here */
void main() {
uint32_t port = 0;
uint32_t pin = 10;
uint32_t pinMask = (1ul << pin);
        init();

        Uart_begin(9600);
	/* Set output direction for LED pin*/
#ifdef IMPROPER
	gpio = (unsigned int*)PORTDIRSET;
	*gpio |= (1 << LED_GPIO_BIT);
#else
      // enable input, to support reading back values, with pullups disabled
      PORT->Group[port].PINCFG[pin].reg = (uint8_t) (PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR);
      // Set pin to output mode
      PORT->Group[port].DIRSET.reg = pinMask;
#endif
#if defined(USE_TINYUSB)
//  TinyUSB_Device_Init(0);
#endif

	/* Never exit */
	while (1) {

        count--;

  if (count <= 1) //10ms * 10000 = 100secs ish, check this works before adding to microbian port for first test recovery
  {
     Uart_end();
     force_bootloader();
  }

//   yield(); // yield run usb background task

  // if (serialEventRun) serialEventRun(); for hardwareserial

  
#if 0
		for (tim = 0; tim < 1000; tim++)
                {
			for (int i =0; i < 1000; i++ ){;}
                        yield();
                }
#else
                Uart_write('X');
                delay(500);
#endif
		/* Clear LED output pin*/
#ifdef IMPROPER
		gpio = (unsigned int*)PORTCLR;
		*gpio = (1 << LED_GPIO_BIT);
#else
                PORT->Group[port].OUTCLR.reg = pinMask;
#endif
 
#if 0
		for (tim = 0; tim < 1000; tim++)
                {
			for (int i =0; i < 1000; i++ ){;}
                        yield();
                }
#else
                Uart_write('Y');
                delay(500);
#endif
		/* Set LED output pin*/
#ifdef IMPROPER
		gpio = (unsigned int*)PORTOUT;
		*gpio = (1 << LED_GPIO_BIT);
#else
                PORT->Group[port].OUTSET.reg = pinMask;
#endif
//        delay(1);//see if we are getting stuck in delay seems to get stuck in delay
	}
}
