#include <stdint.h>
#include <string.h>
#include <ch32v00X.h>

#define IAP_UART_BAUD 115200U

#define RS485_DE_PORT GPIOD
#define RS485_DE_PIN  GPIO_Pin_4

#define LED_ERR_PORT  GPIOC
#define LED_ERR_PIN   GPIO_Pin_6
#define LED_ACT_PORT  GPIOC
#define LED_ACT_PIN   GPIO_Pin_7

#define APP_FLASH_BASE 0x08000000UL
#define IAP_CAL_ADDR   (0x08004000UL - 4UL)
#define IAP_CHECK_NUM  0x5AA55AA5UL

#define UART_SYNC_HEAD1 0xAAU
#define UART_SYNC_HEAD2 0x55U

#define CMD_IAP_PROM   0x80U
#define CMD_IAP_ERASE  0x81U
#define CMD_IAP_VERIFY 0x82U
#define CMD_IAP_END    0x83U
#define CMD_JUMP_IAP   0x84U

#define ERR_SUCCESS 0x00U
#define ERR_ERROR   0x01U
#define ERR_END     0x02U

#define DATA_BUF_SIZE 64U

#define FLASH_KEY1           0x45670123UL
#define FLASH_KEY2           0xCDEF89ABUL
#define FLASH_SR_BSY         0x00000001UL
#define FLASH_CTLR_LOCK_FAST 0x00008000UL

typedef union __attribute__((aligned(4))) {
    struct {
        uint8_t cmd;
        uint8_t len;
        uint8_t data[DATA_BUF_SIZE];
    } uart;
    struct {
        uint8_t raw[DATA_BUF_SIZE + 4U];
    } other;
} isp_cmd_t;

static uint32_t g_program_addr = APP_FLASH_BASE;
static uint32_t g_verify_addr = APP_FLASH_BASE;
static uint8_t g_verify_started = 0U;
static uint8_t g_fast_program_buf[390] __attribute__((aligned(4)));
static uint32_t g_program_buf[64];
static uint16_t g_code_len = 0U;
static uint8_t g_end_flag = 0U;
static uint8_t g_ep_rx_buffer[DATA_BUF_SIZE + 4U] __attribute__((aligned(4)));

#define ISP_CMD ((isp_cmd_t *)g_ep_rx_buffer)

static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles-- != 0U) { }
}

static void leds_init(void)
{
    RCC->PB2PCENR |= RCC_PB2Periph_GPIOC;
    GPIOC->CFGLR &= ~((uint32_t)0xFF000000);
    GPIOC->CFGLR |= (uint32_t)0x33000000;
    GPIOC->BCR = (uint32_t)(LED_ERR_PIN | LED_ACT_PIN);
}

static void led_err_set(uint8_t on)
{
    if (on) GPIOC->BSHR = LED_ERR_PIN;
    else GPIOC->BCR = LED_ERR_PIN;
}

static void led_act_set(uint8_t on)
{
    if (on) GPIOC->BSHR = LED_ACT_PIN;
    else GPIOC->BCR = LED_ACT_PIN;
}

static void rs485_set_tx(uint8_t en)
{
    if (en) GPIOD->BSHR = RS485_DE_PIN;
    else GPIOD->BCR = RS485_DE_PIN;
}

static void usart1_cfg_rs485(void)
{
    RCC->PB2PCENR |= RCC_PB2Periph_GPIOD | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1;
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);
    /* PD6 = USART1 TX (AF_PP), PD5 = USART1 RX (input floating), PD4 = RS485 DE (out PP). */
    GPIOD->CFGLR = 0x4B434444U;
    rs485_set_tx(0U);
    USART1->CTLR2 |= USART_StopBits_1;
    USART1->CTLR1 = 0x200CU;
    USART1->CTLR3 = USART_HardwareFlowControl_None;
#if IAP_UART_BAUD == 460800U
    USART1->BRR = (SystemCoreClock == 24000000U) ? 0x34U : 0x68U;
#elif IAP_UART_BAUD == 115200U
    USART1->BRR = (SystemCoreClock == 24000000U) ? 0xD0U : 0x1A0U;
#else
#error Unsupported IAP_UART_BAUD
#endif
    USART1->CTLR1 |= 0x2000U;
}

static void rs485_send_bytes(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    delay_cycles(96000U);
    rs485_set_tx(1U);
    delay_cycles(200U);
    for (i = 0; i < len; i++) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
        USART_SendData(USART1, buf[i]);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) { }
    rs485_set_tx(0U);
}

static void send_status_frame(uint8_t err)
{
    uint8_t resp[6];
    resp[0] = UART_SYNC_HEAD1;
    resp[1] = UART_SYNC_HEAD2;
    resp[2] = 0x00U;
    resp[3] = err;
    resp[4] = UART_SYNC_HEAD2;
    resp[5] = UART_SYNC_HEAD1;
    rs485_send_bytes(resp, sizeof(resp));
}

static uint8_t uart1_rx_blocking(void)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) { }
    return (uint8_t)(USART_ReceiveData(USART1) & 0xFFU);
}

static void program_buf_modify(void)
{
    uint8_t i;
    for (i = 0; i < 64U; i++) {
        if (i != 63U) {
            g_program_buf[i] = *(uint32_t *)((IAP_CAL_ADDR & 0xFFFFFF00UL) + (4UL * i));
        } else {
            g_program_buf[i] = IAP_CHECK_NUM;
        }
    }
}

static void flash_unlock_fast_local(void)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    FLASH->MODEKEYR = FLASH_KEY1;
    FLASH->MODEKEYR = FLASH_KEY2;
}

static void flash_erase_page_fast_local(uint32_t page_address)
{
    FLASH->CTLR &= ~((uint32_t)(FLASH_CTLR_PAGE_ER | FLASH_CTLR_PAGE_PG));
    FLASH->CTLR |= FLASH_CTLR_PAGE_ER;
    FLASH->ADDR = page_address & 0xFFFFFF00UL;
    FLASH->CTLR |= FLASH_CTLR_STRT;
    while ((FLASH->STATR & FLASH_SR_BSY) != 0U) { }
    FLASH->CTLR &= ~FLASH_CTLR_PAGE_ER;
}

static void flash_erase_all_pages_local(void)
{
    uint32_t addr;
    for (addr = APP_FLASH_BASE; addr < IAP_CAL_ADDR; addr += 0x100UL) {
        flash_erase_page_fast_local(addr);
    }
}

static void system_reset_start_mode_local(uint32_t mode)
{
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    FLASH->BOOT_MODEKEYR = FLASH_KEY1;
    FLASH->BOOT_MODEKEYR = FLASH_KEY2;
    FLASH->STATR &= ~(1UL << 14);
    if (mode != 0U) {
        FLASH->STATR |= (1UL << 14);
    }
    FLASH->CTLR |= FLASH_CTLR_LOCK;
}

static void ch32_iap_program(uint32_t adr, uint32_t *buf)
{
    uint8_t j;

    adr &= 0xFFFFFF00UL;

    FLASH->CTLR |= ((uint32_t)0x00010000);
    FLASH->CTLR |= ((uint32_t)0x00080000);
    while ((FLASH->STATR & ((uint32_t)0x00000001)) != 0U) { }
    FLASH->CTLR &= ~((uint32_t)0x00010000);

    for (j = 0; j < 64U; j++) {
        FLASH->CTLR |= ((uint32_t)0x00010000);
        *(__IO uint32_t *)(adr + 4UL * j) = buf[j];
        FLASH->CTLR |= ((uint32_t)0x00040000);
        while ((FLASH->STATR & ((uint32_t)0x00000001)) != 0U) { }
        FLASH->CTLR &= ~((uint32_t)0x00010000);
    }

    FLASH->CTLR |= ((uint32_t)0x00010000);
    FLASH->ADDR = adr;
    FLASH->CTLR |= ((uint32_t)0x00000040);
    while ((FLASH->STATR & ((uint32_t)0x00000001)) != 0U) { }
    FLASH->CTLR &= ~((uint32_t)0x00010000);
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
        s = ERR_SUCCESS;
        break;

    case CMD_IAP_PROM:
        for (i = 0U; i < length; i++) {
            g_fast_program_buf[g_code_len + i] = ISP_CMD->uart.data[i];
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

    case CMD_IAP_VERIFY:
        if (g_verify_started == 0U) {
            g_verify_started = 1U;
            if (g_code_len != 0U) {
                for (i = 0U; i < (uint8_t)(256U - g_code_len); i++) {
                    g_fast_program_buf[g_code_len + i] = 0xFFU;
                }
                ch32_iap_program(g_program_addr, (uint32_t *)g_fast_program_buf);
                g_code_len = 0U;
            }
        }

        s = ERR_SUCCESS;
        for (i = 0U; i < length; i++) {
            if (ISP_CMD->uart.data[i] != *(uint8_t *)(g_verify_addr + i)) {
                s = ERR_ERROR;
                break;
            }
        }
        g_verify_addr += length;
        break;

    case CMD_IAP_END:
        g_verify_started = 0U;
        g_end_flag = 1U;
        g_program_addr = APP_FLASH_BASE;
        g_verify_addr = APP_FLASH_BASE;
        s = ERR_END;

        flash_erase_page_fast_local(IAP_CAL_ADDR & 0xFFFFFF00UL);
        program_buf_modify();
        ch32_iap_program(IAP_CAL_ADDR & 0xFFFFFF00UL, g_program_buf);

        FLASH->CTLR |= ((uint32_t)0x00008000);
        FLASH->CTLR |= ((uint32_t)0x00000080);
        break;

    case CMD_JUMP_IAP:
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
    uint16_t data_add = 0U;

    if (uart1_rx_blocking() != UART_SYNC_HEAD1) return;
    if (uart1_rx_blocking() != UART_SYNC_HEAD2) return;

    ISP_CMD->uart.cmd = uart1_rx_blocking();
    data_add = (uint16_t)(data_add + ISP_CMD->uart.cmd);
    ISP_CMD->uart.len = uart1_rx_blocking();
    data_add = (uint16_t)(data_add + ISP_CMD->uart.len);

    if (ISP_CMD->uart.cmd == CMD_IAP_ERASE || ISP_CMD->uart.cmd == CMD_IAP_VERIFY) {
        ISP_CMD->other.raw[2] = uart1_rx_blocking();
        data_add = (uint16_t)(data_add + ISP_CMD->other.raw[2]);
        ISP_CMD->other.raw[3] = uart1_rx_blocking();
        data_add = (uint16_t)(data_add + ISP_CMD->other.raw[3]);
        ISP_CMD->other.raw[4] = uart1_rx_blocking();
        data_add = (uint16_t)(data_add + ISP_CMD->other.raw[4]);
        ISP_CMD->other.raw[5] = uart1_rx_blocking();
        data_add = (uint16_t)(data_add + ISP_CMD->other.raw[5]);
    }

    if (ISP_CMD->uart.cmd == CMD_IAP_PROM || ISP_CMD->uart.cmd == CMD_IAP_VERIFY) {
        for (i = 0U; i < ISP_CMD->uart.len; i++) {
            ISP_CMD->uart.data[i] = uart1_rx_blocking();
            data_add = (uint16_t)(data_add + ISP_CMD->uart.data[i]);
        }
    }

    if (uart1_rx_blocking() != (uint8_t)(data_add & 0xFFU)) return;
    if (uart1_rx_blocking() != (uint8_t)(data_add >> 8)) return;
    if (uart1_rx_blocking() != UART_SYNC_HEAD2) return;
    if (uart1_rx_blocking() != UART_SYNC_HEAD1) return;

    led_err_set(1U);
    delay_cycles(2400000U);
    s = rec_data_deal();

    if (s != ERR_END) {
        send_status_frame((s == ERR_ERROR) ? 0x01U : 0x00U);
        led_err_set(s == ERR_ERROR);
    }
}

static void iap_to_app(void)
{
    RCC_ClearFlag();
    system_reset_start_mode_local(0U);
    NVIC_SystemReset();
}

static uint8_t app_is_present(void)
{
    return (*(uint32_t *)APP_FLASH_BASE != 0xFFFFFFFFUL) ? 1U : 0U;
}

static uint8_t boot_mode_requested(void)
{
    return (RCC_GetFlagStatus(RCC_FLAG_SFTRST) == SET) ? 1U : 0U;
}

int main(void)
{
    SystemCoreClockUpdate();
    leds_init();
    usart1_cfg_rs485();
    led_act_set(1U);

    /* Boot the user app by default on power-on/pin reset.
       Stay in IAP only when the app explicitly entered BOOT mode via software reset. */
    if (app_is_present() && !boot_mode_requested()) {
        iap_to_app();
        while (1) { }
    }

    while (1) {
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
            uart_rx_deal();
        }

        if (g_end_flag != 0U) {
            iap_to_app();
            while (1) { }
        }
    }
}
