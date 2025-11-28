/**
 * FUSB302 USB PD Message Sniffer - UART Console Version
 * Target: STM32F072CB (using libopencm3)
 * Functions: UART Console, I2C, and FUSB302 configuration for PD Sniffing.
 */

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

/**
 * @brief Performs a generic I2C write transaction (RegAddr + Data).
 */
static int i2c_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t buffer[2] = {reg_addr, data};
    
    i2c_set_7bit_address(I2C1, FUSB302_ADDR);
    i2c_set_bytes_to_transfer(I2C1, 2);
    i2c_set_write_transfer_dir(I2C1);
    i2c_enable_autoend(I2C1);
    i2c_send_start(I2C1);
    
    for (int i = 0; i < 2; i++) {
        uint32_t timeout = 0x10000;
        while (!((I2C_ISR(I2C1) & I2C_ISR_TXIS) || (I2C_ISR(I2C1) & I2C_ISR_NACKF)) && timeout) { timeout--; }
        if (I2C_ISR(I2C1) & I2C_ISR_NACKF) { i2c_clear_stop(I2C1); i2c_send_stop(I2C1); return -1; }
        if (timeout == 0) { return -1; }
        i2c_send_data(I2C1, buffer[i]);
    }
    
    uint32_t timeout = 0x10000;
    while (!(I2C_ISR(I2C1) & I2C_ISR_STOPF) && timeout) { timeout--; }
    i2c_clear_stop(I2C1);
    
    if (I2C_ISR(I2C1) & I2C_ISR_NACKF) {
        i2c_clear_stop(I2C1);
        uart_printf("I2C NACK on write.\n");
        return -1;
    }
    
    return 0;
}

/**
 * @brief Performs an I2C multi-byte read transaction (RegAddr write + read N bytes).
 * Used here for reading the RX FIFO (FUSB302_REG_FIFOS).
 */
static int i2c_read_multi(uint8_t reg_addr, uint8_t *data, uint8_t len) {
    // 1. Write Register Address (Setup Phase)
    i2c_set_7bit_address(I2C1, FUSB302_ADDR);
    i2c_set_bytes_to_transfer(I2C1, 1);
    i2c_set_write_transfer_dir(I2C1);
    i2c_disable_autoend(I2C1); 
    i2c_send_start(I2C1);
    
    uint32_t timeout = 0x10000;
    while (!((I2C_ISR(I2C1) & I2C_ISR_TXIS) || (I2C_ISR(I2C1) & I2C_ISR_NACKF)) && timeout) { timeout--; }
    if (I2C_ISR(I2C1) & I2C_ISR_NACKF) { i2c_clear_stop(I2C1); i2c_send_stop(I2C1); return -1; }
    i2c_send_data(I2C1, reg_addr);
    
    timeout = 0x10000;
    while (!(I2C_ISR(I2C1) & I2C_ISR_TC) && timeout) { timeout--; }

    // 2. Read Data (Repeated Start Phase)
    i2c_set_7bit_address(I2C1, FUSB302_ADDR);
    i2c_set_bytes_to_transfer(I2C1, len);
    i2c_set_read_transfer_dir(I2C1);
    i2c_enable_autoend(I2C1); 
    i2c_send_start(I2C1);
    
    for (int i = 0; i < len; i++) {
        timeout = 0x10000;
        while (!(I2C_ISR(I2C1) & I2C_ISR_RXNE) && timeout) { timeout--; }
        if (timeout == 0) { uart_printf("I2C Read Timeout.\n"); i2c_send_stop(I2C1); return -1; }
        data[i] = i2c_received_data(I2C1);
    }
    
    // Wait for STOPF (transfer complete)
    timeout = 0x10000;
    while (!(I2C_ISR(I2C1) & I2C_ISR_STOPF) && timeout) { timeout--; }
    i2c_clear_stop(I2C1);

    return 0;
}

static uint8_t i2c_read_reg(uint8_t reg) {
    uint8_t val;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, &val, 1);
    return val;
}

static void i2c_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_transfer7(I2C1, FUSB302_ADDR, buf, 2, NULL, 0);
}

static uint8_t *i2c_read_reg_fifo(uint8_t reg) {
    static uint8_t buf[80];
    size_t nbytes = 80;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, buf, nbytes);
    return buf;
}

static void i2c_write_reg_nbytes(uint8_t reg, const uint8_t *buf, size_t nbytes) {
    uint8_t wbuf_size = 1 + nbytes;
    uint8_t *wbuf = malloc(wbuf_size);
    wbuf[0] = reg;
    memcpy(&wbuf[1], buf, nbytes);
    i2c_transfer7(I2C1, FUSB302_ADDR, wbuf, 1 + nbytes, NULL, 0);
}

static uint8_t i2c_write_read_reg(uint8_t reg, uint8_t val) {
    uint8_t wbuf[2] = {reg, val};
    i2c_write_reg(reg, val);
    uint8_t rval = i2c_read_reg(reg);
    return rval;
}

// --- FUSB302 PD Sniffer Configuration and Monitoring ---

static int fusb302_sniffer_setup(void) {
    uint8_t res;
    
    uart_printf("Initializing FUSB302 for PD Sniffing...\n");
    
    // Reset the FUSB302
    res = i2c_write_read_reg(FUSB302_REG_RESET, FUSB302_RESET_SW);
    uart_printf("FUSB302 RESET reg: 0x%02X\n", res);
    fusb_delay_ms(10);

    // Power on
    res = i2c_write_read_reg(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    uart_printf("FUSB302 POWER reg: 0x%02X\n", res);

    // Enable pull-ups on CC1 and CC2, set default USB current (80µA)
    // Switches0: PU_EN2=1, PU_EN1=1, PDWN2=1, PDWN1=1
    // This enables detection on both CC lines
    res = i2c_write_read_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_PU_EN2 | FUSB302_SW0_PU_EN1 | FUSB302_SW0_PDWN2 | FUSB302_SW0_PDWN1);
    uart_printf("FUSB302 SWITCHES0 reg: 0x%02X\n", res);

    // Set host current advertisement to default USB (80µA)
    // Control0: HOST_CUR[1:0] = 01 (default current)
    res = i2c_write_read_reg(FUSB302_REG_CONTROL0, (1 << FUSB302_CTL0_HOST_CUR_POS));
    uart_printf("FUSB302 CONTROL0 reg: 0x%02X\n", res);

    return 0;
}

/**
 * @brief Checks FUSB302 status and reads any captured PD messages from the FIFO.
 */
static void check_and_read_fifo(void) {
    uart_printf("Checking for PD messages...\n");

    // Read STATUS1 (0x41) to check RX_FULL bit (bit 4) and RX_EMPTY bit (bit 5)
    uint8_t status0 = i2c_read_reg(FUSB302_REG_STATUS1);
    uart_printf("FUSB302 STATUS1: 0x%02X\n", status0);

    // Check if the RX_FULL flag is set (PD message received)
    if (status0 & FUSB302_STATUS1_RX_FULL) {
        uart_printf("\n--- PD Message Captured ---\n");
        uart_printf("Raw FIFO Bytes (HEX): ");

        // Read all available bytes until RX_EMPTY is set.
        while (1) {
            uint8_t *buf = i2c_read_reg_fifo(FUSB302_REG_FIFOS);
            uart_hexdump(buf, 80);

            // Re-read STATUS0 to check for RX_EMPTY 
            if (i2c_read_reg(FUSB302_REG_STATUS1) & FUSB302_STATUS1_RX_EMPTY) {
                uart_printf("Error reading STATUS1 during FIFO read.\n");
                break;
            }
        }
        uart_printf("--- End of PD Message ---\n");
    }
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
    i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

// --- Main Program ---

int main(void) {
    
    // Setup Peripherals
    clock_setup();
    usart_setup();
    i2c_setup();
    
    usart_getc(); // Dummy read to initialize USART

    uart_printf("\n--- FUSB302 PD Message Sniffer Started (UART) ---\n");
    uart_printf("Connect a PD Source/Sink to the USB-C receptacle.\n");
    uart_printf("I2C Address: 0x%02X\n", FUSB302_ADDR);
    
    // Configure FUSB302 for Sniffing
    if (fusb302_sniffer_setup() != 0) {
        uart_printf("FATAL: FUSB302 setup failed. Check I2C connection.\n");
        while(1); // Stop execution
    }

    // Main Loop: Wait for user input to check for PD messages
    while (1) {
        uart_printf("\nPress Enter to check for PD messages...\n");
        usart_getc(); // Dummy read to initialize USART
        // sniff packets
        check_and_read_fifo();
    }
    
    return 0;
}

