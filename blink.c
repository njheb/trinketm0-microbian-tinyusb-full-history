
/*
 *
 * Physical address from SAMD21E18A datasheet
 *
 * */
#include "sam.h"
#include "./microbian/microbian.h"
//#include "lib.h"

//#define IMPROPER
#ifdef IMPROPER
#define GPIO_BASE 0x41004400
#define PORTDIRSET 0x41004408
#define PORTOUT 0x41004410
#define PORTCLR 0x41004414
#define LED_GPIO_BIT 10
#endif

static int A_TASK;
static int B_TASK;
#define PINGPONG 333

/** GPIO Register set */
volatile unsigned int* gpio;

/** Simple loop variable */
volatile unsigned int tim;

extern void force_bootloader(void);
int count = 100;
uint32_t port = 0;
uint32_t pin = 10;

void test_taskA(int arg)
{
uint32_t pinMask = (1ul << pin);

   int client;
   message m;
   //trimer_pulse(500);
   while (1) {
	receive(ANY, &m);
        client = m.sender;
        count--;

  if (count <= 1) //10ms * 10000 = 100secs ish, check this works before adding to microbian port for first t$
  {
     Uart_end();
     force_bootloader();
  }
                Uart_write('X');
                delay(500);
		/* Clear LED output pin*/
#ifdef IMPROPER
		gpio = (unsigned int*)PORTCLR;
		*gpio = (1 << LED_GPIO_BIT);
#else
                PORT->Group[port].OUTCLR.reg = pinMask;
#endif

       // m.int1 = result;
       // send(client, REPLY, &m);
       send(client, PINGPONG, &m);
   }
}

void test_taskB(int arg)
{
uint32_t pinMask = (1ul << pin);

   int client;
   message m;
   //trimer_pulse(500);
   while (1) {

	receive(ANY, &m);
        client = m.sender;

                Uart_write('Y');
                delay(500);
		/* Set LED output pin*/
#ifdef IMPROPER
		gpio = (unsigned int*)PORTOUT;
		*gpio = (1 << LED_GPIO_BIT);
#else
                PORT->Group[port].OUTSET.reg = pinMask;
#endif


       // m.int1 = result;
       // send(client, REPLY, &m);
       send(client, PINGPONG, &m);
   }
}

void test_taskC(int arg)
{
uint32_t pinMask = (1ul << pin);

   int client;
   message m;
   //trimer_pulse(500);
   while (1) {
        count--;

  if (count <= 1) //10ms * 10000 = 100secs ish, check this works before adding $
  {
     Uart_end();
     force_bootloader();
  }
                Uart_write('X');
                delay(500);
                /* Clear LED output pin*/
#ifdef IMPROPER
                gpio = (unsigned int*)PORTCLR;
                *gpio = (1 << LED_GPIO_BIT);
#else
                PORT->Group[port].OUTCLR.reg = pinMask;
#endif


                Uart_write('Y');
                delay(500);
		/* Set LED output pin*/
#ifdef IMPROPER
		gpio = (unsigned int*)PORTOUT;
		*gpio = (1 << LED_GPIO_BIT);
#else
                PORT->Group[port].OUTSET.reg = pinMask;
#endif


   }
}




//see WVariant.h EPortType port = PORTA;
/** Main function - we'll never return from here */
void init() {
uint32_t pinMask = (1ul << pin);

        //arduino_init(); called from __reset() now

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
#if 0
  A_TASK = start("TestA", test_taskA, 0, STACK);
  B_TASK = start("TestB", test_taskB, 0, STACK);
#else
  start("TestC", test_taskC, 0, STACK);
#endif
   //may have to prime this from DefaultSysTick_Handler
   //before a timer is implemented
   //will probably crash if send called from here
//   message m;
//     send(A_TASK, PINGPONG, &m);


}
