/*to be adapted from wiki*/
/*
https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
*/

#define WITHOUT_SAM_H
#include "mcu_pins.h"

#ifdef WITHOUT_SAM_H
#define GPIO_BASE 0x41004400
#define PORTDIRSET 0x41004408
#define PORTOUT 0x41004410
#define PORTCLR 0x41004414
#define LED_GPIO_BIT PIN_REAL_LED
#define DOTSTAR_GPIO_CLK_BIT 1
#define DOTSTAR_GPIO_DAT_BIT 0
#endif

void dotstar_clk_hi(void)
{
   *(unsigned int*)PORTOUT = (1 << DOTSTAR_GPIO_CLK_BIT);
}

void dotstar_clk_lo(void)
{
   *(unsigned int*)PORTCLR = (1 << DOTSTAR_GPIO_CLK_BIT);
}

void dotstar_dat_hi(void)
{
   *(unsigned int*)PORTOUT = (1 << DOTSTAR_GPIO_DAT_BIT);
}

void dotstar_dat_lo(void)
{
   *(unsigned int*)PORTCLR = (1 << DOTSTAR_GPIO_DAT_BIT);
}

void dotstar_init(void)
{
  *(unsigned int*)PORTDIRSET |= (1 << DOTSTAR_GPIO_CLK_BIT);
  *(unsigned int*)PORTDIRSET |= (1 << DOTSTAR_GPIO_DAT_BIT);
  dotstar_clk_lo();
  dotstar_dat_hi();
}

/*
 * Simultaneously transmit and receive a byte on the SPI.
 *
 * Polarity and phase are assumed to be both 0, i.e.:
 *   - input data is captured on rising edge of SCLK.
 *   - output data is propagated on falling edge of SCLK.
 *
 * Returns the received byte.
 */
//uint8_t bitdelay_us = (1000000/ 8000000) / 2; computes to zero
#define SPI_SCLK_LOW_TIME 2
#define SPI_SCLK_HIGH_TIME 2

uint8_t dotstar_transfer_byte(uint8_t byte_out)
{
    uint8_t byte_in = 0;
    uint8_t bit;

    for (bit = 0x80; bit; bit >>= 1) {
        /* Shift-out a bit to the MOSI line */
        if (byte_out & bit)
           dotstar_dat_hi();
        else
           dotstar_dat_lo();

        /* Delay for at least the peer's setup time */
        delayMicroseconds(SPI_SCLK_LOW_TIME);

        /* Pull the clock line high */
        dotstar_clk_hi();
#if 0
        /* Shift-in a bit from the MISO line */
        if (read_MISO() == HIGH)
            byte_in |= bit;
#endif
        /* Delay for at least the peer's hold time */
        delayMicroseconds(SPI_SCLK_HIGH_TIME);

        /* Pull the clock line low */
        dotstar_clk_lo();
    }

    return byte_in;
}


