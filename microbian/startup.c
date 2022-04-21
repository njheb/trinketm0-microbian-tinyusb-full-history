/* common/startup.c */
/* Copyright (c) 2018 J. M. Spivey */

//njh TODO radical changes to hardware.h needed
#include "hardware.h"

/* init -- main program, creates application processes */
void init(void);

void SystemInit(void);
void arduino_init(void);

void default_start(void)
{
    init();                    /* Call the main program. */
    while (1) pause();         /* Halt if init() returns */
}

void __start(void) __attribute((weak, alias("default_start")));

/* The next four routines can be used in C compiler output, even if
not mentioned in the source. */

/* memcpy -- copy n bytes from src to dest (non-overlapping) */
void *memcpy(void *dest, const void *src, unsigned n)
{
    unsigned char *p = dest;
    const unsigned char *q = src;
    while (n-- > 0) *p++ = *q++;
    return dest;
}

/* memmove -- copy n bytes from src to dest, allowing overlaps */
void *memmove(void *dest, const void *src, unsigned n)
{
    unsigned char *p = dest;
    const unsigned char *q = src;
    if (dest <= src)
        while (n-- > 0) *p++ = *q++;
    else {
        p += n; q += n;
        while (n-- > 0) *--p = *--q;
    }
    return dest;
}
    
/* memset -- set n bytes of dest to byte x */
void *memset(void *dest, unsigned x, unsigned n)
{
    unsigned char *p = dest;
    while (n-- > 0) *p++ = x;
    return dest;
}

/* memcmp -- compare n bytes */
int memcmp(const void *pp, const void *qq, int n)
{
    const unsigned char *p = pp, *q = qq;
    while (n-- > 0) {
        if (*p++ != *q++)
            return (p[-1] < q[-1] ? -1 : 1);
    }
    return 0;
}

/* Addresses set by the linker */
extern unsigned char __data_start[], __data_end[],
    __bss_start[], __bss_end[], __etext[], __stack[];

/* __reset -- the system starts here */

#if 1
void __reset(void)
{
//    /* Activate the crystal clock */
//    CLOCK.HFCLKSTARTED = 0;
//    CLOCK.HFCLKSTART = 1;
//    while (! CLOCK.HFCLKSTARTED) { }

    /*already running at 48MHz came from BOOTLOADER*/
    /*vector table pointer pre adjusted to 0x2000 by BOOTLOADER*/
    int data_size = __data_end - __data_start;
    int bss_size = __bss_end - __bss_start;
    memcpy(__data_start, __etext, data_size);
    memset(__bss_start, 0, bss_size);

#if defined(__FPU_USED) && defined(__SAMD51__)
        /* Enable FPU */
        SCB->CPACR |= (0xFu << 20);
        __DSB();
        __ISB();
#endif
    SystemInit(); /*clocks*/
    arduino_init();

    __start();  /*call weak default_start()*/
}
#endif

/* NVIC SETUP FUNCTIONS */

/* On Cortex-M0, only the top two bits of each interrupt priority are
implemented, but for portability priorities should be specified with
integers in the range [0..255].  On Cortex-M4, the top three bits are
implemented.*/


//njh TODO need to checkout this function
/* irq_priority -- set priority for an IRQ to a value [0..255] */
void irq_priority(int irq, unsigned prio)
{
    if (irq < 0)
        SET_BYTE(SCB.SHPR[(irq+12) >> 2], irq & 0x3, prio);
    else
        SET_BYTE(NVIC.IPR[irq >> 2], irq & 0x3, prio);
}
     
/* See hardware.h for macros enable_irq, disable_irq, 
clear_pending, reschedule */

#ifdef ADVANCED_STAGE_OF_DEVELOPMENT 
/* Device register arrays */
volatile _DEVICE _i2c * const I2C[1] = {
    &I2C0
};

volatile _DEVICE _timer * const TIMER[3] = {
    &TIMER0, &TIMER1, &TIMER2
};
#endif

/*  INTERRUPT VECTORS */

/* We use the linker script to define each handler name as an alias
for default_handler if it is not defined elsewhere.  Applications can
subsitute their own definitions for individual handler names like
uart_handler(). */

/* delay_loop -- timed delay */
void delay_loop(unsigned usecs)
{
/*quick and dirty adjust from 16MHz to 48MHz*/
    unsigned t = usecs << 2;
    while (t > 0) {
        /* (500/3)nsec per iteration at 48MHz */
        nop(); nop(); nop();
        t--;
    }
    t = usecs << 2;
    while (t > 0) {
        /* (500/3)nsec per iteration at 48MHz */
        nop(); nop(); nop();
        t--;
    }
    t = usecs << 2;
    while (t > 0) {
        /* (500/3)nsec per iteration at 48MHz */
        nop(); nop(); nop();
        t--;
    }
}

/* spin -- show Seven Stars of Death */
void spin(void)
{
    intr_disable();
#if 0 
    GPIO.DIR = 0xfff0;
    while (1) {
        GPIO.OUT = 0x4000;
        delay_loop(500000);
        GPIO.OUT = 0;
        delay_loop(100000);
    }
#else
/*TODO trinket SPECIFIC should flash trinket BUILTINLED do DOTSTAR later*/
//#define GPIO_BASE 0x41004400
#define PORTDIRSET 0x41004408
#define PORTOUT 0x41004410
#define PORTCLR 0x41004414
#define LED_GPIO_BIT 10
/** GPIO Register set */
    *(unsigned int*)PORTDIRSET |= (1 << LED_GPIO_BIT);

    while (1) {

                *(unsigned int*)PORTOUT = (1 << LED_GPIO_BIT);
        	delay_loop(500000);

                *(unsigned int*)PORTCLR = (1 << LED_GPIO_BIT);
        	delay_loop(100000);

    }
#endif
}

void default_handler(void) __attribute((weak, alias("spin")));

#ifdef MICCROBITSTUFF
//nrf51 specifics in here
/* The linker script makes all these handlers into weak aliases for */
/* default_handler. */

void nmi_handler(void);
void hardfault_handler(void);
void svc_handler(void);
void pendsv_handler(void);
void systick_handler(void);
void uart_handler(void);
void timer0_handler(void);
void timer1_handler(void);
void timer2_handler(void);
void power_clock_handler(void);
void radio_handler(void);
void i2c_handler(void);
void spi_handler(void);
void gpiote_handler(void);
void adc_handler(void);
void rtc0_handler(void);
void temp_handler(void);
void rng_handler(void);
void ecb_handler(void);
void ccm_aar_handler(void);
void wdt_handler(void);
void rtc1_handler(void);
void qdec_handler(void);
void lpcomp_handler(void);
void swi0_handler(void);
void swi1_handler(void);
void swi2_handler(void);
void swi3_handler(void);
void swi4_handler(void);
void swi5_handler(void);

/* This vector table is placed at address 0 in the flash by directives
in the linker script. */

void *__vectors[] __attribute((section(".vectors"))) = {
    __stack,                    /* -16 */
    __reset,
    nmi_handler,
    hardfault_handler,
    0,                          /* -12 */
    0,
    0,
    0,
    0,                          /*  -8 */
    0,
    0,
    svc_handler,
    0,                          /* -4 */
    0,
    pendsv_handler,
    systick_handler,
    
    /* external interrupts */
    power_clock_handler,        /*  0 */
    radio_handler,
    uart_handler,
    i2c_handler,
    spi_handler,                /*  4 */
    0,
    gpiote_handler,
    adc_handler,
    timer0_handler,             /*  8 */
    timer1_handler,
    timer2_handler,
    rtc0_handler,
    temp_handler,               /* 12 */
    rng_handler,
    ecb_handler,
    ccm_aar_handler,
    wdt_handler,                /* 16 */
    rtc1_handler,
    qdec_handler,
    lpcomp_handler,
    swi0_handler,               /* 20 */
    swi1_handler,
    swi2_handler,
    swi3_handler,
    swi4_handler,               /* 24 */
    swi5_handler,
    0,
    0,
    0,                          /* 28 */
    0,
    0,
    0
};
#else

//rely on cortex_handler.c
#endif
