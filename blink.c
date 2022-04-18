
/*
 *
 * Physical address from SAMD21E18A datasheet
 *
 * */
#include "sam.h"
#define GPIO_BASE 0x41004400
#define PORTDIRSET 0x41004408
#define PORTOUT 0x41004410
#define PORTCLR 0x41004414
#define LED_GPIO_BIT 10

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

	/* Set output direction for LED pin*/
//	gpio = (unsigned int*)PORTDIRSET;
//	*gpio |= (1 << LED_GPIO_BIT);
      // enable input, to support reading back values, with pullups disabled
      PORT->Group[port].PINCFG[pin].reg = (uint8_t) (PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR);
      // Set pin to output mode
      PORT->Group[port].DIRSET.reg = pinMask;



	/* Never exit */
	while (1) {
        count--;

  if (count <= 1) //10ms * 10000 = 100secs ish, check this works before adding to microbian port for first test recovery
  {
     force_bootloader();  
  }

		for (tim = 0; tim < 500000; tim++)
			;

		/* Clear LED output pin*/
//		gpio = (unsigned int*)PORTCLR;
//		*gpio = (1 << LED_GPIO_BIT);
                PORT->Group[port].OUTCLR.reg = pinMask;


		for (tim = 0; tim < 500000; tim++)
			;

		/* Set LED output pin*/
		//gpio = (unsigned int*)PORTOUT;
		//*gpio = (1 << LED_GPIO_BIT);
                PORT->Group[port].OUTSET.reg = pinMask;
	}
}
