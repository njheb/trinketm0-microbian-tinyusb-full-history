#include "sam.h"

#if defined(__SAMD51__)
uint32_t SystemCoreClock=F_CPU;
#else
/*
 * System Core Clock is at 1MHz (8MHz/8) at Reset.
 * It is switched to 48MHz in the Reset Handler (startup.c)
 */
uint32_t SystemCoreClock=1000000ul ;
#endif

