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
/**
 * @brief Checks CC lines for device connection and configures FUSB302 accordingly.
 */
static int fusb302_check_cc_lines(void) {
    int ret = 0;
    uint8_t status0 = i2c_read_reg(FUSB302_REG_STATUS0);
    if (status0 & FUSB302_STATUS0_COMP) { // COMP bit indicates something attached
        /*BC_LVL is only defined when Measure block is on which is when
          register bits PWR[2]=1 and either MEAS_CC1=1 or MEAS_CC2=1*/
        uint8_t bc_lvl = status0 & FUSB302_STATUS0_BC_LVL_MASK;
        uart_printf("Device detected on CC lines. BC_LVL: 0x%02X\n", bc_lvl);

        // Determine which CC line has the device
        // Measure CC1 only: MEAS_CC1=1, PU_EN1=1
        i2c_write_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_PU_EN1);
        status0 = i2c_read_reg(FUSB302_REG_STATUS0);
        uart_printf("FUSB302 STATUS0 after CC1 measure: 0x%02X\n", status0);
        uint8_t cc1_level = status0 & FUSB302_STATUS0_BC_LVL_MASK;
        uart_printf("CC1 BC_LVL: 0x%02X\n", cc1_level);

        // Measure CC2 only: MEAS_CC2=1, PU_EN2=1
        i2c_write_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC2 | FUSB302_SW0_PU_EN2);
        status0 = i2c_read_reg(FUSB302_REG_STATUS0);
        uart_printf("FUSB302 STATUS0 after CC2 measure: 0x%02X\n", status0);
        uint8_t cc2_level = status0 & FUSB302_STATUS0_BC_LVL_MASK;
        uart_printf("CC2 BC_LVL: 0x%02X\n", cc2_level);

        // Configure for detected orientation
        if (cc1_level > 0x00 && cc2_level == 0x00) {
            // Return 1 for CC1
            ret = 1;
            // Device on CC1
            uart_printf("Device detected on CC1. BC_LVL: 0x%02X\n", cc1_level);
            // PU_EN1=1, MEAS_CC1=1
            i2c_write_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_PU_EN1 | FUSB302_SW0_MEAS_CC1);
            // TXCC1=1, AUTO_CRC=1
            i2c_write_reg(FUSB302_REG_SWITCHES1, FUSB302_SW1_TXCC1 | FUSB302_SW1_AUTO_CRC);
        } else if (cc2_level > 0x00 && cc1_level == 0x00) {
            // Return 2 for CC2
            ret = 2;
            // Device on CC2
            uart_printf("Device detected on CC2. BC_LVL: 0x%02X\n", cc2_level);
            // PU_EN2=1, MEAS_CC2=1
            i2c_write_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_PU_EN2 | FUSB302_SW0_MEAS_CC2);
            // TXCC2=1, AUTO_CRC=1
            i2c_write_reg(FUSB302_REG_SWITCHES1, FUSB302_SW1_TXCC2 | FUSB302_SW1_AUTO_CRC);
        }
    } else {
        uart_printf("No device detected on CC lines.\n");
        // Return 0 if no device detected
        ret = 0;
    }
    return ret;
}

static void fusb302_sniffer_setup(void) {
    uint8_t res, clear_mask;
    
    uart_printf("Initializing FUSB302 for PD Sniffing...\n");
    
    // Reset the FUSB302
    i2c_write_reg(FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(10);

    // Power on
    i2c_write_reg(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);

    // Unmask all interrupts 
    i2c_write_reg(FUSB302_REG_MASK, 0x00);
    i2c_write_reg(FUSB302_REG_MASKA, 0x00);
    i2c_write_reg(FUSB302_REG_MASKB, 0x00);

    // Configure listening mode
    // Flush RX
    res = i2c_read_reg(FUSB302_REG_CONTROL1);
    res |= FUSB302_CTL1_RX_FLUSH;
    i2c_write_reg(FUSB302_REG_CONTROL1, res);

    // Disable pull-downs
    res = i2c_read_reg(FUSB302_REG_SWITCHES0);
    clear_mask = ~(FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2) & 0xFF;
    res &= clear_mask;
    i2c_write_reg(FUSB302_REG_SWITCHES0, res);
    fusb_delay_ms(10);

    // Enable SOP' and SOP''
    res = i2c_read_reg(FUSB302_REG_CONTROL1);
    res |= (FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);
    i2c_write_reg(FUSB302_REG_CONTROL1, res);

    // Flush TX
    res = i2c_read_reg(FUSB302_REG_CONTROL0);
    res |= FUSB302_CTL0_TX_FLUSH;
    i2c_write_reg(FUSB302_REG_CONTROL0, res);

    // Flush RX again?
    res = i2c_read_reg(FUSB302_REG_CONTROL1);
    res |= FUSB302_CTL1_RX_FLUSH;
    i2c_write_reg(FUSB302_REG_CONTROL1, res);

    // Reset PD
    i2c_write_reg(FUSB302_REG_RESET, FUSB302_RESET_PD);
    fusb_delay_ms(10);

    uart_printf("FUSB302 configured for PD Sniffing.\n");
}

/**
 * @brief Checks FUSB302 status and reads any captured PD messages from the FIFO.
 */
static void check_and_read_fifo(void) {
    uart_printf("Checking for PD messages...\n");
    // I_CRC_CHK bit in INTERRUPT register indicates a received PD message
    uint8_t interrupt = i2c_read_reg(FUSB302_REG_INTERRUPT);
    if (interrupt & FUSB302_INT_CRC_CHK) {
        uart_printf("PD Message Received Interrupt Detected.\n");
        // Read packet from RX FIFO
        // First byte is SOP token
        uint8_t token = i2c_read_reg(FUSB302_REG_FIFOS);
        uart_printf("SOP Token: 0x%02X\n", token);
        uint8_t packet[32];
        uint8_t status1 = i2c_read_reg(FUSB302_REG_STATUS1);
        uart_printf("FUSB302 STATUS1: 0x%02X\n", status1);
        size_t index = 0;

        // While RX_EMPTY == 0
        while (!(status1 & FUSB302_STATUS1_RX_EMPTY)) {
            packet[index++] = i2c_read_reg(FUSB302_REG_FIFOS);
            status1 = i2c_read_reg(FUSB302_REG_STATUS1);
        }
        uart_printf("\n--- PD Message Captured ---\n");
        uart_hexdump(packet, index);
    }
    // Try another method: check STATUS1 for RX_FULL
    // Read STATUS1 (0x41) to check RX_FULL bit (bit 4) and RX_EMPTY bit (bit 5)
    uint8_t status1 = i2c_read_reg(FUSB302_REG_STATUS1);
    uart_printf("FUSB302 STATUS1: 0x%02X\n", status1);

    // Check if the RX_FULL flag is set (PD message received)
    if (status1 & FUSB302_STATUS1_RX_FULL) {
        uart_printf("\n--- PD Message Captured ---\n");
        uart_printf("Raw FIFO Bytes (HEX): ");

        // Read all available bytes until RX_EMPTY is set.
        while (1) {
            uint8_t *buf = i2c_read_reg_fifo(FUSB302_REG_FIFOS);
            uart_printf("--- Packet Hexdump ---\n");
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

// INT I2C Pin not connected by mistake, workaroud...
static void fusb302_poll_fifo(void) {
    // Read all available bytes in bursts
    uart_printf("Polling FIFO for PD messages...\n");
    for (int i = 0; i < 5; i++) {
        uint8_t *buf = i2c_read_reg_fifo(FUSB302_REG_FIFOS);
        // Sum of values in buf to make sure not all zeros
        size_t sum = 0;
        for (size_t j = 0; j < 80; j++) {
            sum += buf[j];
        }
        if (sum > 0) {
            uart_printf("--- Packet Hexdump %d ---\n", i);
            uart_hexdump(buf, 80);
        }
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
    i2c_set_speed(I2C1, i2c_speed_fm_400k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

// --- Main Program ---

int main(void) {
    // Setup Peripherals
    clock_setup();
    usart_setup();
    i2c_setup();

    fusb_delay_ms(100); // Wait for stable power
    usart_getc(); // pause for user

    uart_printf("\n--- FUSB302 PD Message Sniffer Started (UART) ---\n");
    uart_printf("Connect a PD Source/Sink to the USB-C receptacle.\n");
    uart_printf("I2C Address: 0x%02X\n", FUSB302_ADDR);
    
    // Configure FUSB302 for Sniffing
    fusb302_sniffer_setup();
    uart_printf("\nPress Enter to check for PD messages...\n");
    usart_getc(); // pause for user to plug in device

    // Main Loop: Wait for user input to check for PD messages
    while (1) {
        // Check CC lines for device connection
        int dev_detected = fusb302_check_cc_lines();
        if (dev_detected == 0) {
            uart_printf("No device detected. Please connect a PD Source/Sink.\n");
        } else {
            uart_printf("Device detected on CC%d. Monitoring for PD messages...\n", dev_detected);
        }

        // Poll FIFO for any received PD messages
        fusb302_poll_fifo();
        check_and_read_fifo();

        // Delay to avoid busy looping
        fusb_delay_ms(250);
    }

    return 0;
}

