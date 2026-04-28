#include <stdint.h>
#include <ch32v00X.h>

/* Write IAP_CHECK_NUM to IAP_CAL_ADDR so the bootloader always boots the app
   on every subsequent power-on without going through IAP mode. */

#define IAP_CAL_ADDR  (0x08000000UL + 62UL*1024UL - 4UL)  /* 0x0800F7FC */
#define IAP_CHECK_NUM 0x5AA55AA5UL

#define FLASH_KEY1 0x45670123UL
#define FLASH_KEY2 0xCDEF89ABUL

static void flash_unlock_fast(void)
{
    FLASH->KEYR     = FLASH_KEY1; FLASH->KEYR     = FLASH_KEY2;
    FLASH->MODEKEYR = FLASH_KEY1; FLASH->MODEKEYR = FLASH_KEY2;
}

static void flash_erase_page(uint32_t addr)
{
    FLASH->CTLR &= ~((uint32_t)(FLASH_CTLR_PAGE_ER | FLASH_CTLR_PAGE_PG));
    FLASH->CTLR |= FLASH_CTLR_PAGE_ER;
    FLASH->ADDR  = addr & 0xFFFFFF00UL;
    FLASH->CTLR |= FLASH_CTLR_STRT;
    while (FLASH->STATR & 0x01UL) {}
    FLASH->CTLR &= ~FLASH_CTLR_PAGE_ER;
}

static void flash_write_word(uint32_t addr, uint32_t val)
{
    FLASH->CTLR |= 0x00010000UL;
    *(__IO uint32_t *)addr = val;
    FLASH->CTLR |= 0x00040000UL;
    while (FLASH->STATR & 0x01UL) {}
    FLASH->CTLR &= ~0x00010000UL;

    FLASH->CTLR |= 0x00010000UL;
    FLASH->ADDR  = addr & 0xFFFFFF00UL;
    FLASH->CTLR |= 0x00000040UL;
    while (FLASH->STATR & 0x01UL) {}
    FLASH->CTLR &= ~0x00010000UL;
}

int main(void)
{
    SystemCoreClockUpdate();

    if (*(uint32_t *)IAP_CAL_ADDR != IAP_CHECK_NUM) {
        flash_unlock_fast();
        flash_erase_page(IAP_CAL_ADDR);
        flash_write_word(IAP_CAL_ADDR, IAP_CHECK_NUM);
        FLASH->CTLR |= 0x80U;  /* lock */
    }

    while (1) {}
}
