#include "sam.h"

void force_bootloader(void)
{
#if defined(__SAMD51__)
#define BOOT_DOUBLE_TAP_ADDRESS ((volatile uint32_t *)(HSRAM_ADDR + HSRAM_SIZE - 4))
#else
#define BOOT_DOUBLE_TAP_ADDRESS ((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4))
#endif

//#define BOOT_DOUBLE_TAP_ADDRESS           (HSRAM_ADDR + HSRAM_SIZE - 4)
#define BOOT_DOUBLE_TAP_DATA              (*((volatile uint32_t *)BOOT_DOUBLE_TAP_ADDRESS))
#define DOUBLE_TAP_MAGIC                  0xf01669efUL

    /*
         these values are used to call the bootloader on demand
         BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;
         NVIC_SystemReset();      // processor software reset
    A uf2 file can be dragged to the BOOT drive that shows on the PC
     */
     BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;
     NVIC_SystemReset();      // processor software reset

}
