#include <stdint.h>
#include <string.h>
#include <ch32v00X.h>

#define IAP_UART_BAUD  115200U

/* Device identity — change DEVICE_ADDR per unit (0x01–0xFE). */
#define DEV_TYPE_HEATER   0x00U
#define DEV_TYPE_REWINDER 0x01U
#define DEVICE_TYPE       DEV_TYPE_REWINDER
#define DEVICE_ADDR       0x01U
#define BROADCAST_ADDR    0xFFU

#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4
#define FORCE_IAP_PIN GPIO_Pin_1  /* PD1/SWIO: hold low at reset to stay in IAP */

/* Single status LED on PD0. */
#define LED_PORT GPIOD
#define LED_PIN  GPIO_Pin_0

#define APP_FLASH_BASE 0x08000000UL                       /* WCH user flash app base */
#define IAP_CAL_ADDR   (0x08000000UL + 62UL*1024UL - 4UL) /* last word of 62KB flash = 0x0800F7FC */
#define IAP_CHECK_NUM  0x5AA55AA5UL

#define UART_SYNC_HEAD1 0xAAU
#define UART_SYNC_HEAD2 0x55U

/*
 * Protocol request:  AA 55 DEST CMD LEN [DATA×LEN] CSUM_L CSUM_H 55 AA
 *   DEST 0xFF = broadcast (all bootloaders respond, staggered by address)
 *   CSUM = uint16 sum of (DEST + CMD + LEN + DATA bytes)
 *
 * Protocol response: AA 55 DEV_ADDR ERR EXT 55 AA  (7 bytes)
 *   ERR  0x00 = ok, 0x01 = error
 *   EXT  = DEVICE_TYPE for CMD_SCAN, 0x00 otherwise
 */
#define CMD_SCAN       0x01U   /* broadcast-safe device discovery */
#define CMD_IAP_PROM   0x80U
#define CMD_IAP_ERASE  0x81U
#define CMD_IAP_VERIFY 0x82U   /* legacy per-chunk verify, kept for compat */
#define CMD_IAP_END    0x83U
#define CMD_IAP_CRC    0x85U   /* flush+verify: 4-byte payload [size_l size_h crc_l crc_h] */

#define ERR_SUCCESS 0x00U
#define ERR_ERROR   0x01U

#define DATA_BUF_SIZE 128U

#define FLASH_KEY1   0x45670123UL
#define FLASH_KEY2   0xCDEF89ABUL
#define FLASH_SR_BSY 0x00000001UL

typedef union __attribute__((aligned(4))) {
    struct {
        uint8_t cmd;
        uint8_t len;
        uint8_t data[DATA_BUF_SIZE];
    } uart;
} isp_cmd_t;

static uint32_t g_program_addr  = APP_FLASH_BASE;
static uint16_t g_prog_cksum = 0U;  /* running sum of all PROM bytes received */
static uint8_t  g_fast_program_buf[390] __attribute__((aligned(4)));
static uint32_t g_program_buf[64];
static uint16_t g_code_len = 0U;
static uint8_t  g_end_flag = 0U;
static uint8_t  g_ep_rx_buffer[DATA_BUF_SIZE + 4U] __attribute__((aligned(4)));

#define ISP_CMD ((isp_cmd_t *)g_ep_rx_buffer)

static void iap_to_app(void);

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles-- != 0U) {}
}

static void led_init(void)
{
    RCC->PB2PCENR |= RCC_PB2Periph_GPIOD;
    /* PD0 = push-pull output 50 MHz */
    GPIOD->CFGLR = (GPIOD->CFGLR & ~0xFUL) | 0x3UL;
    GPIOD->BCR = LED_PIN;
}

static void led_set(uint8_t on)
{
    if (on) GPIOD->BSHR = LED_PIN;
    else    GPIOD->BCR  = LED_PIN;
}

static void rs485_set_tx(uint8_t en)
{
    if (en) GPIOD->BSHR = RS485_DE_PIN;
    else    GPIOD->BCR  = RS485_DE_PIN;
}

static void usart1_cfg_rs485(void)
{
    RCC->PB2PCENR |= RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1;
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);
    /* PD0 = preserved (LED out), PD1-3 = float in,
       PD4 = PP out (RS485 DE), PD5 = float in (RX), PD6 = AF PP (TX), PD7 = float in */
    GPIOD->CFGLR = (GPIOD->CFGLR & 0x0000000FUL) | 0x4B434440UL;
    rs485_set_tx(0U);
    USART1->CTLR2 |= USART_StopBits_1;
    USART1->CTLR1  = 0x200CU;
    USART1->CTLR3  = USART_HardwareFlowControl_None;
    USART1->BRR    = (SystemCoreClock == 24000000U) ? 0x34U : 0x68U;  /* 460800 baud */
    USART1->CTLR1 |= 0x2000U;
}

static void rs485_send_bytes(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    delay_cycles(96000U);
    rs485_set_tx(1U);
    delay_cycles(200U);
    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
        USART_SendData(USART1, buf[i]);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}
    rs485_set_tx(0U);
}

/* Fixed 7-byte response: AA 55 DEV_ADDR ERR EXT 55 AA */
static void send_resp(uint8_t err, uint8_t ext)
{
    uint8_t resp[7];
    resp[0] = UART_SYNC_HEAD1;
    resp[1] = UART_SYNC_HEAD2;
    resp[2] = DEVICE_ADDR;
    resp[3] = err;
    resp[4] = ext;
    resp[5] = UART_SYNC_HEAD2;
    resp[6] = UART_SYNC_HEAD1;
    rs485_send_bytes(resp, sizeof(resp));
}

static uint8_t force_iap_pin_low(void)
{
    RCC->PB2PCENR |= RCC_PB2Periph_GPIOD;
    /* PD1 input pull-up: MODE=00, CNF=10, ODR=1. */
    GPIOD->CFGLR = (GPIOD->CFGLR & ~(0xFUL << 4)) | (0x8UL << 4);
    GPIOD->BSHR = FORCE_IAP_PIN;
    delay_cycles(1000U);
    return (GPIOD->INDR & FORCE_IAP_PIN) == 0U;
}

static uint8_t uart1_rx_blocking(void)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) {}
    return (uint8_t)(USART_ReceiveData(USART1) & 0xFFU);
}

static void program_buf_modify(void)
{
    uint8_t i;
    for (i = 0; i < 64U; i++) {
        g_program_buf[i] = (i != 63U)
            ? *(uint32_t *)((IAP_CAL_ADDR & 0xFFFFFF00UL) + (4UL * i))
            : IAP_CHECK_NUM;
    }
}

static void flash_unlock_fast_local(void)
{
    FLASH->KEYR    = FLASH_KEY1; FLASH->KEYR    = FLASH_KEY2;
    FLASH->MODEKEYR = FLASH_KEY1; FLASH->MODEKEYR = FLASH_KEY2;
}

static void flash_erase_page_fast_local(uint32_t page_address)
{
    FLASH->CTLR &= ~((uint32_t)(FLASH_CTLR_PAGE_ER | FLASH_CTLR_PAGE_PG));
    FLASH->CTLR |= FLASH_CTLR_PAGE_ER;
    FLASH->ADDR  = page_address & 0xFFFFFF00UL;
    FLASH->CTLR |= FLASH_CTLR_STRT;
    while ((FLASH->STATR & FLASH_SR_BSY) != 0U) {}
    FLASH->CTLR &= ~FLASH_CTLR_PAGE_ER;
}

static void flash_erase_all_pages_local(void)
{
    uint32_t addr;
    for (addr = APP_FLASH_BASE; addr < IAP_CAL_ADDR; addr += 0x400UL) {
        FLASH->CTLR &= ~(uint32_t)FLASH_CTLR_PAGE_ER;
        FLASH->CTLR |= FLASH_CTLR_PER;
        FLASH->ADDR  = addr;
        FLASH->CTLR |= FLASH_CTLR_STRT;
        while ((FLASH->STATR & FLASH_SR_BSY) != 0U) {}
        FLASH->CTLR &= ~FLASH_CTLR_PER;
    }
}


static void ch32_iap_program(uint32_t adr, uint32_t *buf)
{
    uint8_t j;
    adr &= 0xFFFFFF00UL;

    FLASH->CTLR |= 0x00010000UL;
    FLASH->CTLR |= 0x00080000UL;
    while ((FLASH->STATR & 0x00000001UL) != 0U) {}
    FLASH->CTLR &= ~0x00010000UL;

    for (j = 0; j < 64U; j++) {
        FLASH->CTLR |= 0x00010000UL;
        *(__IO uint32_t *)(adr + 4UL * j) = buf[j];
        FLASH->CTLR |= 0x00040000UL;
        while ((FLASH->STATR & 0x00000001UL) != 0U) {}
        FLASH->CTLR &= ~0x00010000UL;
    }

    FLASH->CTLR |= 0x00010000UL;
    FLASH->ADDR  = adr;
    FLASH->CTLR |= 0x00000040UL;
    while ((FLASH->STATR & 0x00000001UL) != 0U) {}
    FLASH->CTLR &= ~0x00010000UL;
}

static uint8_t rec_data_deal(void)
{
    uint8_t i;
    uint8_t s;
    uint8_t length = ISP_CMD->uart.len;

    switch (ISP_CMD->uart.cmd) {
    case CMD_IAP_ERASE:
        flash_unlock_fast_local();
        flash_erase_all_pages_local();
        g_code_len = 0U;
        g_prog_cksum = 0U;
        g_program_addr = APP_FLASH_BASE;
        g_end_flag = 0U;
        s = ERR_SUCCESS;
        break;

    case CMD_IAP_PROM:
        for (i = 0U; i < length; i++) {
            g_fast_program_buf[g_code_len + i] = ISP_CMD->uart.data[i];
            g_prog_cksum = (uint16_t)(g_prog_cksum + ISP_CMD->uart.data[i]);
        }
        g_code_len = (uint16_t)(g_code_len + length);
        if (g_code_len >= 256U) {
            ch32_iap_program(g_program_addr, (uint32_t *)g_fast_program_buf);
            g_code_len = (uint16_t)(g_code_len - 256U);
            for (i = 0U; i < g_code_len; i++) {
                g_fast_program_buf[i] = g_fast_program_buf[256U + i];
            }
            g_program_addr += 0x100UL;
        }
        s = ERR_SUCCESS;
        break;

    case CMD_IAP_CRC:
        /* payload: [cksum_l][cksum_h] — sum of all bytes received via CMD_IAP_PROM */
        {
            uint16_t exp = (uint16_t)ISP_CMD->uart.data[0]
                         | ((uint16_t)ISP_CMD->uart.data[1] << 8);
            s = (g_prog_cksum == exp) ? ERR_SUCCESS : ERR_ERROR;
        }
        break;

    case CMD_IAP_END:
        /* Flush any partial page with 0xFF padding */
        if (g_code_len != 0U) {
            uint16_t j;
            for (j = g_code_len; j < 256U; j++) {
                g_fast_program_buf[j] = 0xFFU;
            }
            ch32_iap_program(g_program_addr, (uint32_t *)g_fast_program_buf);
        }
        g_end_flag = 1U;
        g_code_len = 0U;
        g_prog_cksum = 0U;
        g_program_addr = APP_FLASH_BASE;

        flash_erase_page_fast_local(IAP_CAL_ADDR & 0xFFFFFF00UL);
        program_buf_modify();
        ch32_iap_program(IAP_CAL_ADDR & 0xFFFFFF00UL, g_program_buf);

        FLASH->CTLR |= 0x00008000UL;
        FLASH->CTLR |= 0x00000080UL;
        s = ERR_SUCCESS;
        break;

    default:
        s = ERR_ERROR;
        break;
    }
    return s;
}

static void uart_rx_deal(void)
{
    uint8_t i;
    uint8_t s;
    uint8_t dest;
    uint16_t data_add = 0U;

    if (uart1_rx_blocking() != UART_SYNC_HEAD1) return;
    if (uart1_rx_blocking() != UART_SYNC_HEAD2) return;

    dest = uart1_rx_blocking();
    data_add = dest;

    if (dest != DEVICE_ADDR && dest != BROADCAST_ADDR) return;

    ISP_CMD->uart.cmd = uart1_rx_blocking();
    data_add = (uint16_t)(data_add + ISP_CMD->uart.cmd);
    ISP_CMD->uart.len = uart1_rx_blocking();
    data_add = (uint16_t)(data_add + ISP_CMD->uart.len);

    if (ISP_CMD->uart.len > DATA_BUF_SIZE) return;

    if (ISP_CMD->uart.cmd == CMD_IAP_PROM || ISP_CMD->uart.cmd == CMD_IAP_CRC) {
        for (i = 0U; i < ISP_CMD->uart.len; i++) {
            ISP_CMD->uart.data[i] = uart1_rx_blocking();
            data_add = (uint16_t)(data_add + ISP_CMD->uart.data[i]);
        }
    }

    if (uart1_rx_blocking() != (uint8_t)(data_add & 0xFFU)) return;
    if (uart1_rx_blocking() != (uint8_t)(data_add >> 8))    return;
    if (uart1_rx_blocking() != UART_SYNC_HEAD2) return;
    if (uart1_rx_blocking() != UART_SYNC_HEAD1) return;

    if (ISP_CMD->uart.cmd == CMD_SCAN) {
        /* Stagger broadcast responses by address to avoid RS485 collision. */
        if (dest == BROADCAST_ADDR) {
            delay_cycles((uint32_t)DEVICE_ADDR * 96000U);
        }
        send_resp(ERR_SUCCESS, DEVICE_TYPE);
        return;
    }

    led_set(1U);
    s = rec_data_deal();

    if (!g_end_flag) {
        send_resp((s == ERR_ERROR) ? ERR_ERROR : ERR_SUCCESS, 0x00U);
    }
    led_set(s == ERR_ERROR);

}

static void iap_to_app(void)
{
    /* Clear BOOT_MODE and reset into normal user flash.  Jumping directly to a
       split address is unreliable on this target; use the WCH start-mode flow. */
    FLASH->KEYR          = FLASH_KEY1;
    FLASH->KEYR          = FLASH_KEY2;
    FLASH->BOOT_MODEKEYR = FLASH_KEY1;
    FLASH->BOOT_MODEKEYR = FLASH_KEY2;
    FLASH->STATR        &= ~(uint32_t)FLASH_STATR_BOOT_MODE;
    FLASH->CTLR         |= 0x80U;   /* re-lock */
    RCC_ClearFlag();
    NVIC_SystemReset();
    while (1) {}
}

int main(void)
{
    uint8_t force_iap;
    SystemCoreClockUpdate();
    led_init();
    usart1_cfg_rs485();
    led_set(1U);
    force_iap = force_iap_pin_low();

    /* Boot app only if: valid magic word written by CMD_IAP_END AND power-on reset. */
    if (!force_iap &&
        (*(uint32_t *)IAP_CAL_ADDR == IAP_CHECK_NUM) &&
        (RCC_GetFlagStatus(RCC_FLAG_SFTRST) == RESET)) {
        iap_to_app();
        while (1) {}
    }

    while (1) {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
            uart_rx_deal();
        }
        if (g_end_flag != 0U) {
            iap_to_app();
            while (1) {}
        }
    }
}
