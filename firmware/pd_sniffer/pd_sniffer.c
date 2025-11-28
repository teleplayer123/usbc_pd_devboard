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


// --- FUSB302 PD Sniffer Configuration and Monitoring ---

static int fusb302_sniffer_setup(void) {
    int result;
    
    uart_printf("Initializing FUSB302 for PD Sniffing...\n");
    
    // Reset ALL (masking, PD state, switches, etc)
    if (i2c_write_byte(FUSB302_REG_RESET, FUSB302_POWER_BANDGAP | FUSB302_POWER_RX_REF | FUSB302_POWER_MEAS_BLOCK) != 0) {
        uart_printf("FUSB302 Error: Failed to write RESET register.\n");
        return -1;
    }
    
    // Configure Switches for Passive Sniffing
    // SWITCHES0 (0x02): All off. (CC1_PU/PD, CC2_PU/PD, VBUS, etc.)
    if (i2c_write_byte(FUSB302_REG_SWITCHES0, 0x00) != 0) { result = -2; goto error_exit; }

    // SWITCHES1 (0x03): Set to monitor CC1 AND CC2 for reception.
    // RX_CC1 (bit 0) | RX_CC2 (bit 1) = 0x03
    if (i2c_write_byte(FUSB302_REG_SWITCHES1, 0x03) != 0) { result = -3; goto error_exit; }
    
    // Configure Control Registers
    // CONTROL1 (0x0D): Clear RX_FLUSH (bit 6) to prepare FIFO for new messages.
    if (i2c_write_byte(FUSB302_REG_CONTROL1, 0x00) != 0) { result = -4; goto error_exit; }
    
    // CONTROL2 (0x0E): Set Mode to PD Monitoring (MODE[1:0] = 10b). 
    // This enables the PD core into a listening state (0x02 << 4) = 0x20.
    if (i2c_write_byte(FUSB302_REG_CONTROL2, 0x20) != 0) { result = -5; goto error_exit; }
    
    uart_printf("FUSB302 Sniffer is configured and listening.\n");
    return 0;

error_exit:
    uart_printf("FUSB302 Error during setup sequence: Code %d\n", result);
    return result;
}

/**
 * @brief Checks FUSB302 status and reads any captured PD messages from the FIFO.
 */
static void check_and_read_fifo(void) {
    uint8_t status0;
    
    // 1. Read STATUS1 (0x41) to check RX_FULL bit (bit 4) and RX_EMPTY bit (bit 5)
    // We read STATUS1 multiple times to check the RX_EMPTY state correctly.
    if (i2c_read_multi(FUSB302_REG_STATUS1, &status0, 1) != 0) {
        uart_printf("Error: Cannot read STATUS0.\n");
        return;
    }

    // Check if the RX_FULL flag is set (PD message received)
    if (status0 & FUSB302_STATUS1_RX_FULL) {
        
        uint8_t rx_data_buffer[80]; 
        uint8_t byte_count = 0;
        
        uart_printf("\n--- PD Message Captured ---\n");
        uart_printf("Raw FIFO Bytes (HEX): ");

        // Read all available bytes until RX_EMPTY is set.
        do {
            uint8_t fifo_byte;
            // Read one byte from the FIFO register (0x44)
            if (i2c_read_multi(FUSB302_REG_FIFOS, &fifo_byte, 1) != 0) {
                 uart_printf(" (FIFO Read Fail) ");
                 break;
            }
            rx_data_buffer[byte_count++] = fifo_byte;
            uart_printf("%02X ", fifo_byte);
            
            if (byte_count > 80) { 
                uart_printf(" [Buffer Overflow!] ");
                break; 
            }
            
            // Re-read STATUS0 to check for RX_EMPTY 
            if (i2c_read_multi(FUSB302_REG_STATUS0, &status0, 1) != 0) { break; }
            
        } while (!(status0 & FUSB302_STATUS1_RX_EMPTY)); // Loop while RX_EMPTY is not set
        
        uart_printf("\nTotal Bytes Read: %d\n", byte_count);
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

    // A sniffer passively monitors the two CC lines for traffic. 

    while (1) {
        // Continuously check the FUSB302 for captured messages
        check_and_read_fifo();
    }
    
    return 0;
}

