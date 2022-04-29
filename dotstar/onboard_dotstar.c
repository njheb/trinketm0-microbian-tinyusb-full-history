/*to be adapted from wiki*/
/*
https://en.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
*/
//typedef char uint8_t;
#include "sam.h"
#include "delay.h"
//#define WITHOUT_SAM_H
#include "mcu_pins.h"

#ifdef WITHOUT_SAM_H
#define GPIO_BASE 0x41004400
#define PORTDIRSET 0x41004408
#define PORTOUT 0x41004410
#define PORTCLR 0x41004414
#define LED_GPIO_BIT PIN_REAL_LED
//reverse DOTSTAR bits for itsybitsym0
#endif
#define DOTSTAR_GPIO_CLK_BIT 1
#define DOTSTAR_GPIO_DAT_BIT 0

/*NB looks like need to checkout working of digitalWrite()*/

void dotstar_clk_hi(void)
{
//   *(unsigned int*)PORTOUT = (1 << DOTSTAR_GPIO_CLK_BIT);
     PORT->Group[0].OUTSET.reg = (1 << DOTSTAR_GPIO_CLK_BIT);
}

void dotstar_clk_lo(void)
{
//   *(unsigned int*)PORTCLR = (1 << DOTSTAR_GPIO_CLK_BIT);
     PORT->Group[0].OUTCLR.reg = (1 << DOTSTAR_GPIO_CLK_BIT); 
}

void dotstar_dat_hi(void)
{
//   *(unsigned int*)PORTOUT = (1 << DOTSTAR_GPIO_DAT_BIT);
     PORT->Group[0].OUTSET.reg =  (1 << DOTSTAR_GPIO_DAT_BIT);
}

void dotstar_dat_lo(void)
{
//   *(unsigned int*)PORTCLR = (1 << DOTSTAR_GPIO_DAT_BIT);
   PORT->Group[0].OUTCLR.reg = (1 << DOTSTAR_GPIO_DAT_BIT);
}

void dotstar_init(void)
{
#if 1
// enable input, to support reading back values, with pullups disabled
 PORT->Group[0].PINCFG[DOTSTAR_GPIO_DAT_BIT].reg = (uint8_t) (PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR);
// Set pin to output mode
 PORT->Group[0].DIRSET.reg = 1<<DOTSTAR_GPIO_DAT_BIT;
// enable input, to support reading back values, with pullups disabled
 PORT->Group[0].PINCFG[DOTSTAR_GPIO_CLK_BIT].reg = (uint8_t) (PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR);
// Set pin to output mode
 PORT->Group[0].DIRSET.reg = 1<<DOTSTAR_GPIO_CLK_BIT;
#else
  *(unsigned int*)PORTDIRSET |= (1 << DOTSTAR_GPIO_CLK_BIT); //seems OK without pinMode
  *(unsigned int*)PORTDIRSET |= (1 << DOTSTAR_GPIO_DAT_BIT); //ditto
#endif
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
#define SPI_SCLK_LOW_TIME 1  /*1us works on arduino test code*/
#define SPI_SCLK_HIGH_TIME 1

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


void dotstar_show(int idx)
{                             /*R    G    B */
 const unsigned char spectrum[8][3]={{0x00,0x00,0x00}, //black
			      {0x00,0x00,0x7f}, //blue
			      {0x00,0x3f,0x3f}, //cyan
			      {0x00,0x7f,0x00}, //green
			      {0x3f,0x3f,0x00}, //yellow
			      {0x7f,0x00,0x00}, //red
			      {0x3f,0x00,0x3f}, //magenta
			      {0x2a,0x2a,0x2a}, //white
			      };

//start frame
 (void)dotstar_transfer_byte(0x00);
 (void)dotstar_transfer_byte(0x00);
 (void)dotstar_transfer_byte(0x00);
 (void)dotstar_transfer_byte(0x00);
//pixel
 (void)dotstar_transfer_byte(0xFF);
 (void)dotstar_transfer_byte(spectrum[idx][2]); //Blue confirmed 
 (void)dotstar_transfer_byte(spectrum[idx][1]); //Green by elimination
 (void)dotstar_transfer_byte(spectrum[idx][0]); //Red confirmed
//end frame
 (void)dotstar_transfer_byte(0xFF);

}

void dotstar_show_yellow(void)
{
//start frame
 (void)dotstar_transfer_byte(0x00);
 (void)dotstar_transfer_byte(0x00);
 (void)dotstar_transfer_byte(0x00);
 (void)dotstar_transfer_byte(0x00);
//pixel
 (void)dotstar_transfer_byte(0xFF);
 (void)dotstar_transfer_byte(0x00); //Blue confirmed 
 (void)dotstar_transfer_byte(0x40); //Green by elimination
 (void)dotstar_transfer_byte(0x40); //Red confirmed
//end frame
 (void)dotstar_transfer_byte(0xFF);

}
