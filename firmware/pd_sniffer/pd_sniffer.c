#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "fusb302.h"

/*---- MCU setup functions ----*/

static void clock_setup(void) {
    rcc_clock_setup_in_hsi_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_I2C1);
}

static void usart_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);
    // usart_disable(USART2);
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

static void i2c_setup(void) {
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    // gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO8);

    // Hardware reset via RCC 
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);

    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

static void systick_setup(void) {
    // Set SysTick to trigger every 1ms (48MHz / 1000 = 48000)
    systick_set_reload(48000 - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); // Use AHB clock
    systick_counter_enable();
}

static void exti_setup(void) {
    // FUSB302 INT_N is connected to PB8
    // We need to enable the clock for SYSCFG to configure EXTI.
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);
    
    // Map PB8 to EXTI8
    exti_select_source(EXTI8, GPIOB);

    // Set EXTI8 to trigger on a falling edge (INT_N is active-low)
    exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);

    // Enable EXTI8 interrupt line
    exti_enable_request(EXTI8);

    /* Source Identification: Inside the EXTI4_15_IRQHandler function, you will 
    need to check the specific pending register flag (PR register, bit 8) for EXTI 
    line 8 to determine if it was the source of the interrupt, as lines 4 through 15 
    all share this single handler */
    nvic_enable_irq(NVIC_EXTI4_15_IRQ); 
}

static void usart_send_char(char c) {
    usart_send_blocking(USART2, c);
}

static void usart_printf(const char *format, ...) {
    char buf[2048];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    for (const char *p = buf; *p; p++) {
        usart_send_char(*p);
        // Handle newline conversion for terminal compatibility
        if (*p == '\n') {
            usart_send_char('\r');
        }
    }
}


volatile bool fusb_event_pending = false;
bool pd_monitor = true;
bool caps_received = false;
pd_msg_t last_caps;
int last_pdo_count = 0;

/* ============================================================
 * Low-level helpers (I2C + UART)
 * ============================================================ */

static void fusb_write(uint8_t reg, uint8_t val) {
    i2c_transfer7(I2C1, FUSB302_ADDR << 1, &reg, 1, &val, 1);
}

static uint8_t fusb_read(uint8_t reg) {
    uint8_t v;
    i2c_transfer7(I2C1, FUSB302_ADDR << 1, &reg, 1, &v, 1);
    return v;
}

static void fusb_read_fifo(uint8_t *buf, int len) {
    uint8_t r = FUSB302_REG_FIFOS;
    i2c_transfer7(I2C1, FUSB302_ADDR << 1, &r, 1, buf, len);
}

static bool uart_rx_ready(void) {
    return usart_get_flag(USART2, USART_FLAG_RXNE);
}

/* ============================================================
 * FUSB302 / PD handling
 * ============================================================ */

static bool fusb_rx_empty(void) {
    return fusb_read(FUSB302_REG_STATUS1) & FUSB302_STATUS1_RX_EMPTY;
}

static bool read_pd_message(pd_msg_t *pd) {
    if (fusb_rx_empty()) return false;

    uint8_t hdr[4];
    fusb_read_fifo(hdr, 4);
    pd->header = hdr[2] | (hdr[3] << 8);

    int n = PD_HEADER_NUM_DATA_OBJECTS(pd->header);
    if (n > 0)
        fusb_read_fifo((uint8_t*)pd->obj, n * 4);

    fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_RX_FLUSH);
    return true;
}

static void pd_print_caps(void) {
    usart_printf("\r\n=== Source Capabilities ===\r\n");
    for (int i = 0; i < last_pdo_count; i++) {
        uint32_t o = last_caps.obj[i];
        int mv = ((o >> 10) & 0x3FF) * 50;
        int ma = (o & 0x3FF) * 10;
        usart_printf(" PDO%d: %d mV %d mA\r\n", i + 1, mv, ma);
    }
}

static void pd_send_request(int pdo, int mv, int ma) {
    uint16_t hdr = (1 << 12); /* 1 data object */
    uint32_t rdo = ((pdo + 1) << 28) | ((ma / 10) << 10) | (mv / 50);

    uint8_t tx[20];
    int i = 0;

    tx[i++] = FUSB302_TX_TKN_SOP1;
    tx[i++] = FUSB302_TX_TKN_PACKSYM | 2;
    tx[i++] = hdr & 0xFF;
    tx[i++] = hdr >> 8;

    tx[i++] = FUSB302_TX_TKN_PACKSYM | 4;
    memcpy(&tx[i], &rdo, 4); i += 4;

    tx[i++] = FUSB302_TX_TKN_JAMCRC;
    tx[i++] = FUSB302_TX_TKN_EOP;
    tx[i++] = FUSB302_TX_TKN_TXOFF;
    tx[i++] = FUSB302_TX_TKN_TXON;

    uint8_t r = FUSB302_REG_FIFOS;
    i2c_transfer7(I2C1, FUSB302_ADDR << 1, &r, 1, tx, i);
    fusb_write(FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_START);

    usart_printf("Requesting %d mV %d mA\r\n", mv, ma);
}

static void handle_pd_message(pd_msg_t *p) {
    uint8_t type = PD_HEADER_MESSAGE_TYPE(p->header);

    if (type == 1) { /* Source Capabilities */
        caps_received = true;
        last_caps = *p;
        last_pdo_count = PD_HEADER_NUM_DATA_OBJECTS(p->header);
        pd_print_caps();
    }
    else if (type == 3) usart_printf("<< ACCEPT >>\r\n");
    else if (type == 6) usart_printf("<< PS_RDY >>\r\n");

    if (pd_monitor) {
        usart_printf("PD msg type %d (%d objects)\r\n",
                      type, PD_HEADER_NUM_DATA_OBJECTS(p->header));
    }
}

/* ============================================================
 * CLI
 * ============================================================ */

static void handle_command(const char *cmd) {
    if (!strcmp(cmd, "help")) {
        usart_printf(
            "help\r\ncaps\r\nreq 5|9|15|20\r\nmonitor on|off\r\nreset\r\n");
    }
    else if (!strcmp(cmd, "caps")) {
        if (!caps_received) usart_printf("No caps yet\r\n");
        else pd_print_caps();
    }
    else if (!strncmp(cmd, "req", 3)) {
        int v = atoi(cmd + 4);
        if (!caps_received) return;
        for (int i = 0; i < last_pdo_count; i++) {
            int mv = ((last_caps.obj[i] >> 10) & 0x3FF) * 50;
            if (mv / 1000 == v) {
                pd_send_request(i, mv, 3000);
                return;
            }
        }
        usart_printf("No matching PDO\r\n");
    }
    else if (!strcmp(cmd, "monitor on"))  pd_monitor = true;
    else if (!strcmp(cmd, "monitor off")) pd_monitor = false;
    else if (!strcmp(cmd, "reset"))
        fusb_write(FUSB302_REG_CONTROL3, FUSB302_CTL3_SEND_HARD_RESET);
    else
        usart_printf("Unknown command\r\n");
}

/* ============================================================
 * EXTI ISR
 * ============================================================ */

void exti0_1_isr(void) {
    exti_reset_request(EXTI0);
    fusb_event_pending = true;
}

/* ============================================================
 * Main
 * ============================================================ */

int main(void) {
    clock_setup();
    systick_setup();
    usart_setup();
    i2c_setup();
    exti_setup();

    usart_printf("---- PD Debugger ----\r\n> ");

    char line[32];
    int pos = 0;

    while (1) {

        /* UART CLI (non-blocking) */
        if (uart_rx_ready()) {
            char c = usart_recv(USART2);
            if (c == '\r' || c == '\n') {
                line[pos] = 0;
                usart_printf("\r\n");
                handle_command(line);
                pos = 0;
                usart_printf("> ");
            } else if (pos < (int)sizeof(line) - 1) {
                usart_send_blocking(USART2, c);
                line[pos++] = c;
            }
        }

        /* USB-PD handling */
        if (fusb_event_pending) {
            fusb_event_pending = false;
            while (!fusb_rx_empty()) {
                pd_msg_t msg;
                if (read_pd_message(&msg))
                    handle_pd_message(&msg);
            }
        }
    }
}