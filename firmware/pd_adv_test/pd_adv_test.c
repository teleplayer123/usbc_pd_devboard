#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "fusb302.h"

// --- UART Console Functions ---

/* simple blocking getchar/putchar */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}

/**
 * @brief Sends a single character over USART2 (blocking).
 */
static void usart_send_char(char c) {
    usart_send_blocking(USART2, c);
}

/**
 * @brief Custom printf equivalent using USART2.
 */
static void uart_printf(const char *format, ...) {
    char buf[128];
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

static char usart_getc(void) { 
    return usart_recv_blocking(USART2); 
}

static void uart_hexdump(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uart_printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            uart_printf("\n");
        }
    }
    uart_printf("\n");
}

void fusb_delay_ms(uint32_t ms) {
    for (volatile uint32_t i=0; i<ms*4800; i++);
}

// --- I2C Communication Functions ---

static uint8_t fusb_read_reg(uint8_t reg) {
    uint8_t val;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, &val, 1);
    return val;
}

static void fusb_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_transfer7(I2C1, FUSB302_ADDR, buf, 2, NULL, 0);
}


// Helper to write a 32-bit PDO into the FIFO
static void fusb_write_pdo(uint32_t pdo) {
    fusb_write_reg(FUSB302_REG_FIFOS, pdo & 0xFF);
    fusb_write_reg(FUSB302_REG_FIFOS, (pdo >> 8) & 0xFF);
    fusb_write_reg(FUSB302_REG_FIFOS, (pdo >> 16) & 0xFF);
    fusb_write_reg(FUSB302_REG_FIFOS, (pdo >> 24) & 0xFF);
}

void send_source_capabilities(void) {
    // Build PD header: Source_Capabilities, 2 data objects, spec rev 2, power role = Source
    uint16_t header = 0;
    header |= (1 << 8);        // Message type = Source_Capabilities (data message type 1)
    header |= (2 << 6);        // Spec revision = 2 (PD2.0)
    header |= (1 << 5);        // Power role = Source
    header |= (2 << 12);       // Number of data objects = 2

    // Example PDOs
    uint32_t pdo1 = ((5000/50) << 10) | ((3000/10) << 0); // 5V @ 3A
    uint32_t pdo2 = ((9000/50) << 10) | ((2000/10) << 0); // 9V @ 2A

    // Flush TX FIFO before loading
    fusb_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_FLUSH);

    // Sequence of tokens + data into FIFO
    fusb_write_reg(FUSB302_REG_FIFOS, FUSB302_TX_TKN_SOP1); // SOP token
    fusb_write_reg(FUSB302_REG_FIFOS, FUSB302_TX_TKN_PACKSYM | (2 + 2*4)); // header(2) + 2 PDOs(8)

    // Header (2 bytes)
    fusb_write_reg(FUSB302_REG_FIFOS, header & 0xFF);
    fusb_write_reg(FUSB302_REG_FIFOS, (header >> 8) & 0xFF);

    // PDOs
    fusb_write_pdo(pdo1);
    fusb_write_pdo(pdo2);

    // CRC + End of Packet + TXON
    fusb_write_reg(FUSB302_REG_FIFOS, FUSB302_TX_TKN_JAMCRC);
    fusb_write_reg(FUSB302_REG_FIFOS, FUSB302_TX_TKN_EOP);
    fusb_write_reg(FUSB302_REG_FIFOS, FUSB302_TX_TKN_TXON);
}

static void fusb_init(void) {
    fusb_write_reg(FUSB302_REG_SWITCHES1, FUSB302_SW1_POWERROLE | FUSB302_SW1_AUTO_CRC | FUSB302_SW1_TXCC1);
}

// --- Peripheral Setup ---

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

    /* Hardware reset via RCC */
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);

    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

// --- Main Program ---

int main(void) {
    // Setup Peripherals
    clock_setup();
    usart_setup();
    i2c_setup();

    fusb_init();
    uart_printf("FUSB302 Initialized\n");
    while (1) {
        uart_printf("Sending Source Capabilities...\n");
        send_source_capabilities();
        fusb_delay_ms(3000);
    }
    return 0;
}

