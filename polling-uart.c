#if 0
  // D3, ADC, PWM, IRQ, UART RX, Captouch and general purpose pin
  { PORTA,  7, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel7, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, // TCC1/W$

  // D4, ADC, PWM, IRQ, UART TX, Captouch and general purpose pin
  { PORTA,  6, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel6, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6 }, // TCC1/W$



//from variant.cpp
SERCOM sercom0( SERCOM0 ) ;
Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

//void SERCOM0_Handler()
//{
//  Serial1.IrqHandler();
//}
#endif

//not dealing with __SAMD51_ in constructor, probably only SPI specific_
//from constructor equivalent
Sercom* s = SERCOM0;

/* =========================
 * ===== Sercom UART
 * =========================
*/
void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{
  initClockNVIC();
  resetUART();

  //Setting the CTRLA register
  sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(mode) |
                            SERCOM_USART_CTRLA_SAMPR(sampleRate);

  //Setting the Interrupt register
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  //Received complete
                               SERCOM_USART_INTENSET_ERROR; //All others errors

  if ( mode == UART_INT_CLOCK )
  {
    uint16_t sampleRateValue;

    if (sampleRate == SAMPLE_RATE_x16) {
      sampleRateValue = 16;
    } else {
      sampleRateValue = 8;
    }

    // Asynchronous fractional mode (Table 24-2 in datasheet)
    //   BAUD = fref / (sampleRateValue * fbaud)
    // (multiply by 8, to calculate fractional piece)
#if defined(__SAMD51__)
    uint32_t baudTimes8 = (SERCOM_FREQ_REF * 8) / (sampleRateValue * baudrate);
#else
    uint32_t baudTimes8 = (SystemCoreClock * 8) / (sampleRateValue * baudrate);
#endif

    sercom->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    sercom->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
  }
}
void SERCOM::initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |=
    SERCOM_USART_CTRLA_FORM((parityMode == SERCOM_NO_PARITY ? 0 : 1) ) |
    dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

  //Setting the CTRLB register
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(charSize) |
    nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
    (parityMode == SERCOM_NO_PARITY ? 0 : parityMode) <<
      SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
}

void SERCOM::initPads(SercomUartTXPad txPad, SercomRXPad rxPad)
{
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(txPad) |
                             SERCOM_USART_CTRLA_RXPO(rxPad);

  // Enable Transceiver and Receiver
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
}

void SERCOM::resetUART()
{
  // Start the Software Reset
  sercom->USART.CTRLA.bit.SWRST = 1 ;

  while ( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
  {
    // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
  }
}

void SERCOM::enableUART()
{
  //Setting  the enable bit to 1
  sercom->USART.CTRLA.bit.ENABLE = 0x1u;

  //Wait for then enable bit from SYNCBUSY is equal to 0;
  while(sercom->USART.SYNCBUSY.bit.ENABLE);
}

void SERCOM::flushUART()
{
  // Skip checking transmission completion if data register is empty
  if(isDataRegisterEmptyUART())
    return;

  // Wait for transmission to complete
  while(!sercom->USART.INTFLAG.bit.TXC);
}

void SERCOM::clearStatusUART()
{
  //Reset (with 0) the STATUS register
  sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
}

bool SERCOM::availableDataUART()
{
  //RXC : Receive Complete
  return sercom->USART.INTFLAG.bit.RXC;
}

bool SERCOM::isUARTError()
{
  return sercom->USART.INTFLAG.bit.ERROR;
}

void SERCOM::acknowledgeUARTError()
{
  sercom->USART.INTFLAG.bit.ERROR = 1;
}

bool SERCOM::isBufferOverflowErrorUART()
{
  //BUFOVF : Buffer Overflow
  return sercom->USART.STATUS.bit.BUFOVF;
}

bool SERCOM::isFrameErrorUART()
{
  //FERR : Frame Error
  return sercom->USART.STATUS.bit.FERR;
}

void SERCOM::clearFrameErrorUART()
{
  // clear FERR bit writing 1 status bit
  sercom->USART.STATUS.bit.FERR = 1;
}

bool SERCOM::isParityErrorUART()
{
  //PERR : Parity Error
  return sercom->USART.STATUS.bit.PERR;
}

bool SERCOM::isDataRegisterEmptyUART()
{
  //DRE : Data Register Empty
  return sercom->USART.INTFLAG.bit.DRE;
}

uint8_t SERCOM::readDataUART()
{
  return sercom->USART.DATA.bit.DATA;
}

int SERCOM::writeDataUART(uint8_t data)
{
  // Wait for data register to be empty
  while(!isDataRegisterEmptyUART());

  //Put data into DATA register
  sercom->USART.DATA.reg = (uint16_t)data;
  return 1;
}

void SERCOM::enableDataRegisterEmptyInterruptUART()
{
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

void SERCOM::disableDataRegisterEmptyInterruptUART()
{
  sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

//more code available at the bottom of  SERCOM.cpp

//pick up code from Uart.cpp - does some pinmode stuff

