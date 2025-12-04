#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/systick.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "fusb302.h" 

// ============================================================================
// Global Variables and I2C/UART/Delay Functions
// ============================================================================

// simple blocking getchar/putchar
int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}

// A simple delay function (blocking)
static void delay_ms(uint32_t ms) {
    // This assumes SysTick is running at 1ms intervals.
    for (uint32_t i = 0; i < ms; i++) {
        // Wait for the SysTick flag to be set (1ms elapsed)
        while ((STK_CSR & STK_CSR_COUNTFLAG) == 0);
    }
}

// Low-level I2C Write function
static int fusb302_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_buf[2] = {reg, val};
    i2c_transfer7(I2C1, FUSB302_ADDR, tx_buf, 2, NULL, 0);
    // Simple blocking transfer, ignoring error checking for brevity
    return 0;
}

// Low-level I2C Read function (reads 1 byte)
static uint8_t fusb302_read_reg(uint8_t reg) {
    uint8_t rx_val = 0;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, &rx_val, 1);
    return rx_val;
}

// Sends a single character over USART2 (blocking).
static void usart_send_char(char c) {
    usart_send_blocking(USART2, c);
}

// Custom printf equivalent using USART2.
static void usart_printf(const char *format, ...) {
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

static void usart_hexdump(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        usart_printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            usart_printf("\n");
        }
    }
    usart_printf("\n");
}

static char usart_getc(void) { 
    return usart_recv_blocking(USART2); 
}

// ============================================================================
// Peripheral Initialization
// ============================================================================

static void systick_setup(void) {
    // Set SysTick to trigger every 1ms (48MHz / 1000 = 48000)
    systick_set_reload(48000 - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); // Use AHB clock
    systick_counter_enable();
}

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
    usart_disable(USART2);
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
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO8);
    /* Hardware reset via RCC */
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);

    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
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

// ============================================================================
// FUSB302 Core Functions
// ============================================================================

static void fusb302_reset(void) {
    // 1. Perform a Software Reset (clears state machines and FIFOs)
    fusb302_write_reg(FUSB302_REG_RESET, FUSB302_RESET_SW); 
    delay_ms(2);
    
    // 2. Clear all masks to enable *all* interrupts initially (for debugging)
    fusb302_write_reg(FUSB302_REG_MASK, 0x00);
    fusb302_write_reg(FUSB302_REG_MASKA, 0x00);
    fusb302_write_reg(FUSB302_REG_MASKB, 0x00);
    
    // 3. Clear all pending interrupts by reading them
    fusb302_read_reg(FUSB302_REG_INTERRUPT);
    fusb302_read_reg(FUSB302_REG_INTERRUPTA);
    fusb302_read_reg(FUSB302_REG_INTERRUPTB);
}

static void fusb302_init(void) {
    // Read Device ID to confirm FUSB302 is alive
    if (fusb302_read_reg(FUSB302_REG_DEVICE_ID) != 0x91) {
        usart_printf("FUSB302 ID check failed (Expected 0x91).\r\n");
        delay_ms(5);
        // Fall through to try initialization anyway.
    }

    // Perform full reset
    fusb302_reset();
    delay_ms(5);
    // 1. Power up all non-VCONN blocks (Bandgap, Rx/Tx, Measure, Internal Osc)
    // The FUSB302 requires setting Bits 0, 1, 2, 3 for full power.
    fusb302_write_reg(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    
    // 2. Set up SWITCHES0: Enable PDWN1/PDWN2 (to listen)
    // Clear all PUs/VCONN/MEAS and just enable PDWN
    fusb302_write_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2); 
    
    // 3. Set up SWITCHES1: Set DFP (Source) role and enable Auto CRC
    // This is a passive sniffer, so DRP mode is typically set via CONTROL2 toggle.
    // Use DFP mode for immediate configuration and enable Auto CRC
    fusb302_write_reg(FUSB302_REG_SWITCHES1, FUSB302_SW1_AUTO_CRC | FUSB302_SW1_DATAROLE);

    // 4. Set up CONTROL1: Enable listening for SOP and Hard Reset
    fusb302_write_reg(FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2); 
    
    // 5. Set up CONTROL2: Set Dual-Role Power (DRP) Toggle Mode
    // DRP mode (2) and enable TOGGLE (Bit 0) to start DRP discovery.
    uint8_t ctrl2_val = FUSB302_CTL2_MODE_DRP | FUSB302_CTL2_TOGGLE;
    fusb302_write_reg(FUSB302_REG_CONTROL2, ctrl2_val);

    // 6. Set up Masks for Sniffing (We only care about received packets)
    // MASKA: Enable Hard Reset (HARDRST) interrupt.
    fusb302_write_reg(FUSB302_REG_MASKA, ~FUSB302_MASKA_HARDRST); // Clear mask for HARDRST (Bit 0)

    // MASKB: Enable Good CRC Sent (GCRCSENT) interrupt. This confirms an Rx message.
    fusb302_write_reg(FUSB302_REG_MASKB, ~FUSB302_MASKB_GCRCSENT); // Clear mask for GCRCSENT (Bit 0)

    // MASK: Enable COMP_CHNG (Comparator Change) interrupt for CC detection
    fusb302_write_reg(FUSB302_REG_MASK, ~FUSB302_MASK_COMP_CHNG); // Clear mask for COMP_CHNG (Bit 5)
    
    usart_printf("FUSB302 configured for DRP/Sniffing.\r\n");
    delay_ms(1000);
}

static void fusb302_handle_rx_packet(void) {
    uint8_t rx_byte;
    uint8_t rx_buf[64] = {0}; // Max PD message size is 28 bytes + tokens/CRC
    uint8_t len = 0;
    
    // Read the entire RX FIFO (starting at FUSB302_REG_FIFOS, 0x43)
    usart_printf("RX_PKT: ");

    // Read until RX_EMPTY (Status1 bit 5) is set
    while (!(fusb302_read_reg(FUSB302_REG_STATUS1) & FUSB302_STATUS1_RX_EMPTY)) {
        // We read from the same FIFOS register address repeatedly
        rx_byte = fusb302_read_reg(FUSB302_REG_FIFOS);
        
        if (len < sizeof(rx_buf)) {
            rx_buf[len++] = rx_byte;
        }
    }

    // Flush RX FIFO 
    fusb302_write_reg(FUSB302_REG_CONTROL1, FUSB302_CTL1_RX_FLUSH);

    // Print hexdump of received packet
    if (len > 0) {
        usart_printf("RX Packet Length: %d bytes\r\n", len);
        usart_printf("RX Packet Data:\r\n");
        usart_hexdump(rx_buf, len);
        delay_ms(5);
    } else {
        usart_printf("RX Packet Length: 0 bytes\r\n");
    }
}

// ============================================================================
// Interrupt Handler
// ============================================================================

// Handler for EXTI4_15_IRQ (handles PB8 interrupt)
void exti4_15_isr(void) {
    if (exti_get_flag_status(EXTI8)) {
        // Read the Interrupt registers to see what happened and clear the interrupt.
        uint8_t int_a = fusb302_read_reg(FUSB302_REG_INTERRUPTA);
        uint8_t int_b = fusb302_read_reg(FUSB302_REG_INTERRUPTB);
        uint8_t int_c = fusb302_read_reg(FUSB302_REG_INTERRUPT);

        // --- Hard Reset Received / Sent ---
        if (int_a & FUSB302_INTA_HARDRST) {
            usart_printf("INT: Hard Reset Detected.\r\n");
            // Hard Reset requires clearing state and re-toggling DRP.
            fusb302_reset();
        }

        // --- PD Packet Received ---
        // A received message is confirmed when GoodCRC is sent (GCRCSENT)
        if (int_b & FUSB302_INTB_GCRCSENT) {
            usart_printf("INT: GoodCRC Sent (Packet Received Confirmation).\r\n");
            // The packet is waiting in the FIFO
            fusb302_handle_rx_packet();
        }
        
        // --- CC Comparator Change ---
        if (int_c & FUSB302_INT_COMP_CHNG) {
            uint8_t status0 = fusb302_read_reg(FUSB302_REG_STATUS0);
            uint8_t bc_lvl = (status0 & FUSB302_STATUS0_BC_LVL_MASK) >> FUSB302_STATUS0_BC_LVL_POS;
            usart_printf("INT: CC Change (BC_LVL=%02X). Status0: %02X", bc_lvl, status0);
            /* Note:
               Could re-configure the CC lines based on the detected level (if in DFP/UFP mode).
               For a passive sniffer, we mostly log the event and maintain DRP toggling. */
        }

        // Clear the EXTI pending bit *after* handling the events
        exti_reset_request(EXTI8); 
    }
}

// ============================================================================
// Main Loop
// ============================================================================

int main(void) {
    clock_setup();
    systick_setup();

    // 1. Initialize Peripherals
    usart_setup();
    usart_getc(); 
    i2c_setup();
    exti_setup();

    // 2. Initialize FUSB302
    fusb302_init();

    // Loop indefinitely
    while (1) {
        // The main loop does very little; all PD traffic is handled in the 
        // EXTI interrupt service routine (ISR) driven by the FUSB302 INT_N pin.
        delay_ms(500);
    }

    return 0;
}