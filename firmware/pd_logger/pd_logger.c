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
#include <stdint.h>
#include <stdbool.h>
#include "fusb302.h"

/* ------------------------------------------------------------
 * Globals
 * ------------------------------------------------------------ */

volatile uint32_t system_millis;

/* ------------------------------------------------------------
 * MCU Setup Functions
 * ------------------------------------------------------------ */

void sys_tick_handler(void)
{
    system_millis++;
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

static void systick_setup(void)
{
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
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

/* ------------------------------------------------------------
 * USART logging functions
 * ------------------------------------------------------------ */

/* simple blocking getchar/putchar */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}

static void usart_send_char(char c) {
    usart_send_blocking(USART2, c);
}

static char usart_getc(void) { 
    return usart_recv_blocking(USART2); 
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

static void print_byte_as_bits(uint8_t byte, uint8_t reg) {
    usart_printf("Reg %02X: ", reg);
    for (int i = 7; i >= 0; i--) { // Loop from most significant bit (7) to least significant (0)
        // Use a bitwise AND with a mask to check if the current bit is set
        // The mask is 1 shifted left by 'i' positions
        if ((byte >> i) & 1) { 
            usart_printf("1");
        } else {
            usart_printf("0");
        }
    }
    usart_printf("\r\n");
}

static void hexdump(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i += 16) {
        usart_printf("%04X: ", (unsigned int)i);
        
        // Print hex bytes
        for (size_t j = 0; j < 16 && i + j < len; j++) {
            usart_printf("%02X ", data[i + j]);
        }
        
        // Padding for alignment
        for (size_t j = len - i; j < 16; j++) {
            usart_printf("   ");
        }
        
        // Print ASCII representation
        usart_printf(" | ");
        for (size_t j = 0; j < 16 && i + j < len; j++) {
            uint8_t c = data[i + j];
            usart_printf("%c", (c >= 32 && c < 127) ? c : '.');
        }
        usart_printf("\r\n");
    }
}

static inline bool uart_rx_ready(void)
{
    return usart_get_flag(USART2, USART_FLAG_RXNE);
}

/* ------------------------------------------------------------
 * I2C read/write functions
 * ------------------------------------------------------------ */

static void fusb_write(uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2] = {reg, val};
    i2c_transfer7(I2C1, FUSB302_ADDR, tx_buf, 2, NULL, 0);
}

static inline uint8_t fusb_read(uint8_t reg)
{
    uint8_t v;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, &v, 1);
    return v;
}

static inline void fusb_read_fifo(uint8_t *buf, int len)
{
    uint8_t r = FUSB302_REG_FIFOS;
    i2c_transfer7(I2C1, FUSB302_ADDR, &r, 1, buf, len);
}

/* ------------------------------------------------------------
 * FUSB302 functions
 * ------------------------------------------------------------ */

static inline bool fusb_rx_empty(void)
{
    return fusb_read(FUSB302_REG_STATUS1) & FUSB302_STATUS1_RX_EMPTY;
}

static void fusb_delay_ms(uint32_t ms) {
    // This assumes SysTick is running at 1ms intervals.
    for (uint32_t i = 0; i < ms; i++) {
        // Wait for the SysTick flag to be set (1ms elapsed)
        while ((STK_CSR & STK_CSR_COUNTFLAG) == 0);
    }
}

static void fusb_reset(void) {
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(2);
}

static void fusb_pd_reset(void) {
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_PD);
    fusb_delay_ms(2);
}

static void fusb_full_reset(void) {
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_SW | FUSB302_RESET_PD);
    fusb_delay_ms(2);
}

static void fusb_power_all(void) {
    fusb_write(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(1);
}

static void fusb_init_sink(void) {
    // Reset FUSB302
    fusb_reset();
    // Power on all blocks
    fusb_power_all();
    // Enable Auto-CRC, Set sink role
    fusb_write(FUSB302_REG_SWITCHES1, FUSB302_SW1_AUTO_GCRC | FUSB302_SW1_SPECREV1 | FUSB302_SW1_SPECREV0);
    // Toggle to detect CC and establish UFP (Sink)
    fusb_write(FUSB302_REG_CONTROL2, FUSB302_CTL2_MODE_UFP | FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE);
    usart_printf("FUSB302 initialized in Sink mode.\n");
}

static void fusb_setup_sniffer(void) {    
    usart_printf("Initializing FUSB302 for PD Sniffing...\n");
    
    // Reset the FUSB302
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_SW | FUSB302_RESET_PD);
    fusb_delay_ms(2);

    // Power on
    fusb_write(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(2);

     // Configure Switches0: Enable measurement (passive detection) on CC1 and CC2
    fusb_write(FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2 | FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);

    // Configure Control1: Enable reception of all SOP packets
    fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);

    // Accept SOP packets
    fusb_write(FUSB302_REG_CONTROL2, FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE);

    // Unmask all interrupts
    fusb_write(FUSB302_REG_MASK, 0x00);
    fusb_write(FUSB302_REG_MASKA, 0x00);
    fusb_write(FUSB302_REG_MASKB, 0x00);

    // Clear interrupts
    read_reg(FUSB302_REG_INTERRUPT);
    read_reg(FUSB302_REG_INTERRUPTA);
    read_reg(FUSB302_REG_INTERRUPTB);
    fusb_delay_ms(2);

    usart_printf("FUSB302 configured for PD Sniffing.\n");
}

static void check_rx_buffer(void) {
    uint8_t rx_buffer[80];
    fusb_read_fifo(rx_buffer, 80);
    hexdump(rx_buffer, 80);
}

static void fusb_get_status(void) {
    uint8_t st0 = fusb_read(FUSB302_REG_STATUS0);
    uint8_t st1 = fusb_read(FUSB302_REG_STATUS1);
    usart_printf("FUSB STATUS0=0x%02X STATUS1=0x%02X\r\n", st0, st1);
    check_rx_buffer();
    usart_printf("INT pin=%02X\r\n", gpio_get(GPIOB, GPIO8) ? 1 : 0);
}

static int fusb_measure_cc_pin_src(uint8_t cc_reg) {
    // Read status from switches0 register
    uint8_t reg, sw0_orig, cc_lvl;
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    sw0_orig = reg;
    // Clear measurement bits
    reg &= ~(FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);
    // Set measurement bit for desired CC pin
    if (cc_reg == FUSB302_SW0_MEAS_CC1) {
        reg |= FUSB302_SW0_PU_EN1;  // Measure CC1
    } else if (cc_reg == FUSB302_SW0_MEAS_CC2) {
        reg |= FUSB302_SW0_PU_EN2;  // Measure CC2
    }
    // Set CC measure bit
    reg |= cc_reg;
    // Set measurement switch
    fusb_write(FUSB302_REG_SWITCHES0, reg);
    // Set MDAC to default value
    uint8_t mdac = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    fusb_write(FUSB302_REG_MEASURE, mdac);
    fusb_delay_ms(2);
    // Read status register
    reg = fusb_read(FUSB302_REG_STATUS0);
    // Assume open
    cc_lvl = 0;
    // CC voltage below no connect threshold
    if ((reg & FUSB302_STATUS0_COMP) == 0) {
        fusb_write(FUSB302_REG_MEASURE, PD_SRC_DEF_RD_MV);
        fusb_delay_ms(2);

        // Read status register
        reg = fusb_read(FUSB302_REG_STATUS0);

        cc_lvl = (reg & FUSB302_STATUS0_COMP) ? TYPEC_CC_VOLT_RD : TYPEC_CC_VOLT_RA;
    }
    // Restore original switches0 register
    fusb_write(FUSB302_REG_SWITCHES0, sw0_orig);
    return cc_lvl;
}

static uint8_t fusb_measure_cc_pin_snk(void) {
    uint8_t reg, orig_cc1, orig_cc2, bc_lvl_cc1, bc_lvl_cc2, cc_lvl;

    // Measure cc1
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    if (reg & FUSB302_SW0_MEAS_CC1) {
        orig_cc1 = 1;
    } else {
        orig_cc1 = 0;
    }
    if (reg & FUSB302_SW0_MEAS_CC2) {
        orig_cc2 = 1;
    } else {
        orig_cc2 = 0;
    }

    // Disable cc2 measurement switch, enable cc1 measurement switch
    reg &= ~FUSB302_SW0_MEAS_CC2;
    reg |= FUSB302_SW0_MEAS_CC1;
    fusb_write(FUSB302_REG_SWITCHES0, reg);
    // Wait for measurement
    fusb_delay_ms(2);
    // Read cc1 measurement
    bc_lvl_cc1 = fusb_read(FUSB302_REG_STATUS0);
    // Mask unwanted bits
    bc_lvl_cc1 &= (FUSB302_STATUS0_BC_LVL0 | FUSB302_STATUS0_BC_LVL1);
    usart_printf("CC1 Sink BC_LVL: %02X\r\n", bc_lvl_cc1);

    // Measure cc2
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    // Disable cc1 measurement switch and enable cc2 measurement switch
    reg &= ~FUSB302_SW0_MEAS_CC1;
    reg |= FUSB302_SW0_MEAS_CC2;
    fusb_write(FUSB302_REG_SWITCHES0, reg);
    // Wait on measurement
    fusb_delay_ms(2);
    // Read cc2 measurement
    bc_lvl_cc2 = fusb_read(FUSB302_REG_STATUS0);
    // Mask unwanted bits
    bc_lvl_cc2 &= (FUSB302_STATUS0_BC_LVL0 | FUSB302_STATUS0_BC_LVL1);
    usart_printf("CC2 Sink BC_LVL: %02X\r\n", bc_lvl_cc2);

    if (bc_lvl_cc1 > bc_lvl_cc2) {
        cc_lvl = bc_lvl_cc1;
    } else {
        cc_lvl = bc_lvl_cc2;
    }

    // Reset MEAS switches to original state
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    if (orig_cc1) {
        reg |= FUSB302_SW0_MEAS_CC1;
    } else {
        reg &= ~FUSB302_SW0_MEAS_CC1;
    }
    if (orig_cc2) {
        reg |= FUSB302_SW0_MEAS_CC2;
    } else {
        reg &= ~FUSB302_SW0_MEAS_CC2;
    }
    fusb_write(FUSB302_REG_SWITCHES0, reg);

    return cc_lvl;
}

static void fusb_enable_gcrc(bool enable) {
    // AUTO_GCRC is in SWITCHES1 register
    uint8_t reg = fusb_read(FUSB302_REG_SWITCHES1);
    if (enable) {
        reg |= FUSB302_SW1_AUTO_GCRC;
    } else {
        reg &= ~FUSB302_SW1_AUTO_GCRC;
    }
}

static int fusb_check_cc_lines_src(void) {
    int ret = 0;
    fusb_power_all();
    int cc1_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC1);
    int cc2_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC2);
    if (cc1_lvl != TYPEC_CC_VOLT_OPEN && cc2_lvl == TYPEC_CC_VOLT_OPEN) {
        ret = 1; // Device detected on CC1
    } else if (cc2_lvl != TYPEC_CC_VOLT_OPEN && cc1_lvl == TYPEC_CC_VOLT_OPEN) {
        ret = 2; // Device detected on CC2
    }
    return ret;
}

int main(void) {
    clock_setup();
    systick_setup();
    usart_setup();
    i2c_setup();
    exti_setup(); 

    fusb_init_sink();

    while (1) {
        uint8_t cc_lvl = fusb_measure_cc_pin_snk();
        usart_printf("CC Level: %02X\r\n", cc_lvl);
        usart_printf("Enter to continue...\r\n");
        usart_getc();
    }
}