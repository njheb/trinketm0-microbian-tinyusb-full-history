#if defined( ADAFRUIT_TRINKET_M0 )
#define PIN_REAL_LED 10
#define PIN_REAL_RX 7
#define PIN_REAL_TX 6
#define UART_PIO PIO_SERCOM_ALT
#define PIN_REAL_DOTSTAR_DAT 0
#define PIN_REAL_DOTSTAR_CLK 1
#endif

#if defined( ADAFRUIT_ITSYBITSY_M0 )
#define PIN_REAL_LED 17
#define PIN_REAL_RX 11
#define PIN_REAL_TX 10
#define UART_PIO PIO_SERCOM
#define PIN_REAL_DOTSTAR_DAT 1
#define PIN_REAL_DOTSTAR_CLK 0
#endif

#ifndef PIN_REAL_TX
#error "Pins not defined for board or board not defined"
#endif
