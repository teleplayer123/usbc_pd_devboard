#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "fusb302.h"

// Status flag for exti handler
volatile bool fusb_event_pending = false;

static struct fusb302_chip_state {
	int cc_polarity;
	int vconn_enabled;
	/* 1 = pulling up (DFP) 0 = pulling down (UFP) */
	int pulling_up;
	int rx_enable;
	uint8_t mdac_vnc;
	uint8_t mdac_rd;
} state;

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

/*---- UART Functions ----*/

/* simple blocking getchar/putchar */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}

static char usart_getc(void) { 
    return usart_recv_blocking(USART2); 
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

static void dump_bits(uint8_t reg, const struct bit_name *tbl)
{
    for (int i = 0; i <= 7; i++) {
        if (tbl[i].name == NULL)
            continue;  // skip unused bits             

        usart_printf("%s = %d\n", tbl[i].name, !!(reg & tbl[i].mask));
    }
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

static bool uart_rx_ready(void) {
    return usart_get_flag(USART2, USART_FLAG_RXNE);  // RX buffer not empty
}

/*---- I2C functions ----*/
/*
void i2c_transfer7(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn);
    i2c: The base address of the I2C peripheral
    addr: The 7-bit I2C slave address
    w: A pointer to the data buffer to be written to the slave
    wn: The number of bytes to write
    r: A pointer to the buffer where the data read from the slave will be stored
    rn: The number of bytes to read from the slave
*/

static void fusb_read_reg(uint32_t i2c, uint8_t reg, uint8_t *val) {
    i2c_transfer7(i2c, FUSB302_ADDR, &reg, 1, val, 1);
}
static void fusb_write_reg(uint32_t i2c, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_transfer7(i2c, FUSB302_ADDR, buf, 2, NULL, 0);
}

static void fusb_read_reg_nbytes(uint32_t i2c, uint8_t reg, uint8_t *buf, size_t nbytes) {
    i2c_transfer7(i2c, FUSB302_ADDR, &reg, 1, buf, nbytes);
}

static void fusb_write_reg_nbytes(uint32_t i2c, uint8_t reg, const uint8_t *buf, size_t nbytes) {
    uint8_t wbuf_size = 1 + nbytes;
    uint8_t *wbuf = malloc(wbuf_size);
    wbuf[0] = reg;
    memcpy(&wbuf[1], buf, nbytes);
    i2c_transfer7(i2c, FUSB302_ADDR, wbuf, 1 + nbytes, NULL, 0);
}

static void fusb_read_fifo(uint8_t *data, size_t len) {
    uint8_t reg = FUSB302_REG_FIFOS;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, data, len);
}

/* low level i2c scan */
static bool i2c_probe_addr(uint32_t i2c, uint8_t addr) {

    /* clear flags */
    I2C_ICR(i2c) = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    /* send address */
    I2C_CR2(i2c) =
        (addr << 1) |      // address in bits 7:1
        (0 << 16)  |       // number of bytes
        I2C_CR2_START;     // generate START

    /* wait for either ACK or NACK */
    while (1) {
        uint32_t isr = I2C_ISR(i2c);

        if (isr & I2C_ISR_NACKF) {
            I2C_ICR(i2c) = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
            return false;  // NACK means no device
        }

        if (isr & I2C_ISR_STOPF) {
            I2C_ICR(i2c) = I2C_ICR_STOPCF;
            return true;   // STOP with no NACK means device responded
        }
    }
}

static uint8_t i2c_read_reg(uint8_t reg) {
    uint8_t val;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, &val, 1);
    return val;
}

static void i2c_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_buf[2] = {reg, val};
    i2c_transfer7(I2C1, FUSB302_ADDR, tx_buf, 2, NULL, 0);
}

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

/*---- FUSB302 functions ----*/

// Print current state struct values for debugging
static void fusb_current_state(void)
{
    usart_printf("---- Current FUSB302 State ----\r\n");
    usart_printf("CC Polarity: %d\r\n", state.cc_polarity);
    usart_printf("VCONN Enabled: %d\r\n", state.vconn_enabled);
    if (state.pulling_up) {
        usart_printf("Pulling Up (DFP): %d\r\n", state.pulling_up);
    } else {
        usart_printf("Pulling Down (UFP): %d\r\n", state.pulling_up);
    }
    usart_printf("RX Enable: %d\r\n", state.rx_enable);
    usart_printf("MDAC VNC: 0x%02X\r\n", state.mdac_vnc);
    usart_printf("MDAC RD: 0x%02X\r\n", state.mdac_rd);
    usart_printf("---- End State ----\r\n");
}

static void fusb_delay_ms(uint32_t ms) {
    // This assumes SysTick is running at 1ms intervals.
    for (uint32_t i = 0; i < ms; i++) {
        // Wait for the SysTick flag to be set (1ms elapsed)
        while ((STK_CSR & STK_CSR_COUNTFLAG) == 0);
    }
}

static void fusb_delay_us(uint32_t us) {
    // At 48MHz, approximately 48 clock cycles per microsecond
    // Using a simple busy-wait loop with NOP instructions
    // Each NOP takes 1 cycle, loop overhead is minimal
    for (uint32_t i = 0; i < us * 6; i++) {
        __asm__("nop");
    }
}

static void fusb_reset(uint32_t i2c) {
    fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(10);
}

static void fusb_pd_reset(uint32_t i2c) {
    fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_PD);
    fusb_delay_ms(10);
}

static void fusb_full_reset(uint32_t i2c) {
    fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW | FUSB302_RESET_PD);
    fusb_delay_ms(10);
}

static void fusb_power_all(uint32_t i2c) {
    fusb_write_reg(i2c, FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(1);
}

static uint8_t fusb_get_chip_id(uint32_t i2c) {
    uint8_t id;
    fusb_read_reg(i2c, FUSB302_REG_DEVICE_ID, &id);
    return id;
}

static int convert_bc_lvl(int bc_lvl, bool is_src)
{
    int tc_lvl = TYPEC_CC_VOLT_OPEN;
    if (is_src) {
        if (bc_lvl == 0x00) {
            tc_lvl = TYPEC_CC_VOLT_RA;
        } else if (bc_lvl < 0x03) {
            tc_lvl = TYPEC_CC_VOLT_RD;
        }
    } else {
        if (bc_lvl == 0x01) {
            tc_lvl = TYPEC_CC_VOLT_SNK_DEF;
        } else if (bc_lvl == 0x02) {
            tc_lvl = TYPEC_CC_VOLT_SNK_1_5;
        } else if (bc_lvl == 0x03) {
            tc_lvl = TYPEC_CC_VOLT_SNK_3_0;
        }
    }
    return tc_lvl;
}

static int fusb_measure_cc_pin_src(uint8_t cc_reg)
{
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
    fusb_delay_us(250);
    // Read status register
    reg = fusb_read(FUSB302_REG_STATUS0);
    // Assume open
    cc_lvl = 0;
    // CC voltage below no connect threshold
    if ((reg & FUSB302_STATUS0_COMP) == 0) {
        fusb_write(FUSB302_REG_MEASURE, PD_SRC_DEF_RD_MV);
        fusb_delay_us(250);

        // Read status register
        reg = fusb_read(FUSB302_REG_STATUS0);

        cc_lvl = (reg & FUSB302_STATUS0_COMP) ? TYPEC_CC_VOLT_RD : TYPEC_CC_VOLT_RA;
    }
    // Restore original switches0 register
    fusb_write(FUSB302_REG_SWITCHES0, sw0_orig);
    return cc_lvl;
}

static void fusb_measure_cc_pin_snk(int *cc1, int *cc2)
{
    uint8_t reg, orig_cc1, orig_cc2, bc_lvl_cc1, bc_lvl_cc2;

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
    fusb_delay_us(250);
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
    fusb_delay_us(250);
    // Read cc2 measurement
    bc_lvl_cc2 = fusb_read(FUSB302_REG_STATUS0);
    // Mask unwanted bits
    bc_lvl_cc2 &= (FUSB302_STATUS0_BC_LVL0 | FUSB302_STATUS0_BC_LVL1);
    usart_printf("CC2 Sink BC_LVL: %02X\r\n", bc_lvl_cc2);

    *cc1 = convert_bc_lvl(bc_lvl_cc1, false);
    *cc2 = convert_bc_lvl(bc_lvl_cc2, false);

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
}

static void fusb_enable_gcrc(uint32_t i2c, bool enable) {
    uint8_t reg;
    // AUTO_GCRC is in SWITCHES1 register
    fusb_read_reg(i2c, FUSB302_REG_SWITCHES1, &reg);
    if (enable) {
        reg |= FUSB302_SW1_AUTO_GCRC;
    } else {
        reg &= ~FUSB302_SW1_AUTO_GCRC;
    }
}

static void fusb_flush_rx(uint32_t i2c) {
    uint8_t res;
    // Flush RX
    fusb_read_reg(i2c, FUSB302_REG_CONTROL1, &res);
    res |= FUSB302_CTL1_RX_FLUSH;
    fusb_write_reg(i2c, FUSB302_REG_CONTROL1, res);
}

static void fusb_flush_tx(uint32_t i2c) {
    uint8_t res;
    // Flush TX
    fusb_read_reg(i2c, FUSB302_REG_CONTROL0, &res);
    res |= FUSB302_CTL0_TX_FLUSH;
    fusb_write_reg(i2c, FUSB302_REG_CONTROL0, res);
}

static void fusb_init_sink(uint32_t i2c) {
    // Reset FUSB302
    fusb_full_reset(i2c);
    // Power on all blocks
    fusb_power_all(i2c);
    // Enable Auto-CRC, Set sink role
    fusb_write_reg(i2c, FUSB302_REG_SWITCHES1, FUSB302_SW1_AUTO_GCRC | FUSB302_SW1_SPECREV1 | FUSB302_SW1_SPECREV0);
    // Toggle to detect CC and establish UFP (Sink)
    fusb_write_reg(i2c, FUSB302_REG_CONTROL2, FUSB302_CTL2_MODE_UFP | FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE);
    usart_printf("FUSB302 initialized in Sink mode.\n");
}

// static void fusb_setup_sniffer(int32_t i2c) {    
//     usart_printf("Initializing FUSB302 for PD Sniffing...\n");
    
//     // Reset the FUSB302
//     fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW | FUSB302_RESET_PD);
//     fusb_delay_ms(2);

//     // Power on
//     fusb_write_reg(i2c, FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
//     fusb_delay_ms(2);

//      // Configure Switches0: Enable measurement (passive detection) on CC1 and CC2
//     fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2 | FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);

//     // Configure Control1: Enable reception of all SOP packets
//     fusb_write_reg(i2c, FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);

//     // Accept SOP packets
//     fusb_write_reg(i2c, FUSB302_REG_CONTROL2, FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE);

//     // Unmask all interrupts
//     fusb_write_reg(i2c, FUSB302_REG_MASK, 0x00);
//     fusb_write_reg(i2c, FUSB302_REG_MASKA, 0x00);
//     fusb_write_reg(i2c, FUSB302_REG_MASKB, 0x00);

//     // Clear interrupts
//     i2c_read_reg(FUSB302_REG_INTERRUPT);
//     i2c_read_reg(FUSB302_REG_INTERRUPTA);
//     i2c_read_reg(FUSB302_REG_INTERRUPTB);
//     fusb_delay_ms(2);

//     usart_printf("FUSB302 configured for PD Sniffing.\n");
// }

static void fusb_setup_sniffer(void)
{    
    uint8_t reg;

    usart_printf("Initializing FUSB302 for PD Sniffing...\n");

    state.mdac_vnc = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    state.mdac_rd = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_RD_MV);
    
    // Reset the FUSB302
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(2);

    // Power on
    fusb_write(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(1);

    // Configure Switches0: Enable measurement (passive detection) on CC1 and CC2
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    reg |= (FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2 | FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);
    fusb_write(FUSB302_REG_SWITCHES0, reg);

    // Configure Control1: Enable reception of all SOP packets
    fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);

    // Enable Auto-CRC, Set sink role
    fusb_write(FUSB302_REG_SWITCHES1, FUSB302_SW1_AUTO_GCRC | FUSB302_SW1_SPECREV1 | FUSB302_SW1_SPECREV0);

    // Toggle to detect CC and establish UFP (Sink)
    fusb_write(FUSB302_REG_CONTROL2, FUSB302_CTL2_MODE_UFP | FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE);

    // Unmask all interrupts
    fusb_write(FUSB302_REG_MASK, 0x00);
    fusb_write(FUSB302_REG_MASKA, 0x00);
    fusb_write(FUSB302_REG_MASKB, 0x00);

    // Clear interrupts
    fusb_read(FUSB302_REG_INTERRUPT);
    fusb_read(FUSB302_REG_INTERRUPTA);
    fusb_read(FUSB302_REG_INTERRUPTB);
    fusb_delay_ms(2);

    // Set VCONN and polarity defaults
    state.vconn_enabled = 0;
    state.cc_polarity = 0;
    // Pull-down enabled
    state.pulling_up = 0;
    // RX enabled
    state.rx_enable = 1;

    usart_printf("FUSB302 configured for PD Sniffing.\n");
}

static void fusb_setup(void)
{
    uint8_t reg;

    state.mdac_vnc = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    state.mdac_rd = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_RD_MV);

    // Reset the FUSB302
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(2);

    // Power on
    fusb_write(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(1);

    // Turn on retries and set number of retries
    reg = fusb_read(FUSB302_REG_CONTROL3);
    reg |= (FUSB302_CTL3_AUTO_RETRY | FUSB302_CTL3_NRETRIES_MASK);
    fusb_write(FUSB302_REG_CONTROL3, reg);

    // Create interrupt masks
    reg = 0xFF;
    // CC level changes
    reg &= ~FUSB302_MASK_BC_LVL;
    // Collisions
    reg &= ~FUSB302_MASK_COLLISION;
    // Alert
    reg &= ~FUSB302_MASK_ALERT;
    // Packet received with correct crc
    reg &= ~FUSB302_MASK_CRC_CHK;
    fusb_write(FUSB302_REG_MASK, reg);

    // MaskA reg masks
    reg = 0xFF;
    reg &= ~FUSB302_MASKA_RETRYFAIL;
    reg &= ~FUSB302_MASKA_HARDSENT;
    reg &= ~FUSB302_MASKA_TXSENT;
    reg &= ~FUSB302_MASKA_HARDRST;
    fusb_write(FUSB302_REG_MASKA, reg);
    
    // Mask GoodCRC to ack pd message
    reg = 0xFF;
    reg &= ~FUSB302_MASKB_GCRCSENT;
    fusb_write(FUSB302_REG_MASKB, reg);

    // Enable interrupt
    reg = fusb_read(FUSB302_REG_CONTROL0);
    reg &= ~FUSB302_CTL0_INT_MASK;
    fusb_write(FUSB302_REG_CONTROL0, reg);

    // Set VCONN and polarity defaults
    state.vconn_enabled = 0;
    state.cc_polarity = 0;
}

static bool fusb_rx_empty(void) {
    return (i2c_read_reg(FUSB302_REG_STATUS1) & FUSB302_STATUS1_RX_EMPTY);
}

static void fusb_check_status_regs(void)
{
    uint8_t reg;
    // Read and print status0 bits
    usart_printf("---- STATUS0 ----\r\n");
    reg = fusb_read(FUSB302_REG_STATUS0);
    dump_bits(reg, fusb302_status0_bits);
    usart_printf("\r\n");
    // Read and print status1 bits
    usart_printf("---- STATUS1 ----\r\n");
    reg = fusb_read(FUSB302_REG_STATUS1);
    dump_bits(reg, fusb302_status1_bits);
    usart_printf("\r\n");
    // Read and print status0a bits
    usart_printf("---- STATUS0A ----\r\n");
    reg = fusb_read(FUSB302_REG_STATUS0A);
    dump_bits(reg, fusb302_status0a_bits);
    usart_printf("\r\n");
    // Read and print status1a bits
    usart_printf("---- STATUS1A ----\r\n");
    reg = fusb_read(FUSB302_REG_STATUS1A);
    dump_bits(reg, fusb302_status1a_bits);
    usart_printf("\r\n");
}

static void check_rx_buffer(void)
{
    uint8_t rx_buffer[80];
    fusb_read_fifo(rx_buffer, 80);
    hexdump(rx_buffer, 80);
}

// function for debugging info
static int fusb_check_cc_pin_src(void)
{
    int ret = 0;
    int cc1_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC1);
    int cc2_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC2);
    if (cc1_lvl != TYPEC_CC_VOLT_OPEN && cc2_lvl == TYPEC_CC_VOLT_OPEN) {
        ret = 1; // Device detected on CC1
    } else if (cc2_lvl != TYPEC_CC_VOLT_OPEN && cc1_lvl == TYPEC_CC_VOLT_OPEN) {
        ret = 2; // Device detected on CC2
    }
    return ret;
}

static void fusb_detect_cc_pin_src(int *cc1, int *cc2)
{
    uint8_t cc1_meas = FUSB302_SW0_MEAS_CC1;
    uint8_t cc2_meas = FUSB302_SW0_MEAS_CC2;

    if (state.vconn_enabled) {
        // measure pin matching polarity
        if (state.cc_polarity) {
            // cc2 pin
            *cc2 = fusb_measure_cc_pin_src(cc2_meas);
        } else {
            // cc1 pin
            *cc1 = fusb_measure_cc_pin_src(cc1_meas);
        }
    } else {
        // measure both cc pins if vconn not enabled
        *cc1 = fusb_measure_cc_pin_src(cc1_meas);
        *cc2 = fusb_measure_cc_pin_src(cc2_meas);
    }
}

static void fusb_get_cc(int *cc1, int *cc2)
{
    if (state.pulling_up) {
        // source
        fusb_detect_cc_pin_src(cc1, cc2);
    } else {
        // sink
        fusb_measure_cc_pin_snk(cc1, cc2);
    }
}

// function for debugging info
static void fusb_check_cc_pin_snk(void)
{
    int cc1, cc2;
    fusb_measure_cc_pin_snk(&cc1, &cc2);
    usart_printf("Sink CC1: 0x%02X CC2: 0x%02X\r\n", cc1, cc2);
}

static int fusb_set_cc(int pull)
{
    uint8_t reg;

    switch (pull) {
        case TYPEC_CC_RP:
            usart_printf("Setting CC to type RP...\r\n");
            reg = fusb_read(FUSB302_REG_SWITCHES0);
            // enable needed pull-up
            reg &= ~(FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2 | FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2 | FUSB302_SW0_VCONN_CC1 | FUSB302_SW0_VCONN_CC2);
            reg |= (FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2);

            if (state.vconn_enabled) {
                reg |= state.cc_polarity ? FUSB302_SW0_VCONN_CC1 : FUSB302_SW0_VCONN_CC2;
            }
            fusb_write(FUSB302_REG_SWITCHES0, reg);
            state.pulling_up = 1;
            break;
        case TYPEC_CC_RD:
            usart_printf("Setting CC to type RD...\r\n");
            // enable UFP mode
            // turn off toggle
            reg = fusb_read(FUSB302_REG_CONTROL2);
            reg &= ~(FUSB302_CTL2_TOGGLE);
            fusb_write(FUSB302_REG_CONTROL2, reg);
            
            // enable pull-downs and disable pull-ups
            reg = fusb_read(FUSB302_REG_SWITCHES0);
            reg &= ~(FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2);
            reg |= (FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);
            fusb_write(FUSB302_REG_SWITCHES0, reg);
            state.pulling_up = 0;
            break;
        case TYPEC_CC_OPEN:
            usart_printf("Setting CC to type Open...\r\n");
            // disable toggle
            reg = fusb_read(FUSB302_REG_CONTROL2);
            reg &= ~(FUSB302_CTL2_TOGGLE);
            fusb_write(FUSB302_REG_CONTROL2, reg);
            // manual switches must be open
            reg = fusb_read(FUSB302_REG_SWITCHES0);
            reg &= ~(FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2 | FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);
            fusb_write(FUSB302_REG_SWITCHES0, reg);
            state.pulling_up = 0;
            break;
        default:
            usart_printf("Unsupported CC type!\r\n");
            // unsupported
            return -1;
    }
    return 0;
}

static void fusb_sop_prime_enable(void)
{
    int reg = fusb_read(FUSB302_REG_CONTROL1);
    reg |= (FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2);
    fusb_write(FUSB302_REG_CONTROL1, reg);
}

static void fusb_sop_prime_disable(void)
{
    int reg = fusb_read(FUSB302_REG_CONTROL1);
    reg &= ~(FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2);
    fusb_write(FUSB302_REG_CONTROL1, reg);
}

// function to print status info for debugging
static void fusb_get_status(void)
{
    check_rx_buffer();
    fusb_check_status_regs();
    usart_printf("INT pin=%02X\r\n", gpio_get(GPIOB, GPIO8) ? 1 : 0);
    if (state.pulling_up) {
        // source
        uint8_t cc_pin = fusb_check_cc_pin_src();
        usart_printf("Source CC pin: %02X\r\n", cc_pin);
    } else {
        // sink
        fusb_check_cc_pin_snk();
    }
    fusb_current_state();
}

/* ---- PD Functions ----*/

bool read_pd_message(pd_msg_t *pd)
{
    if (fusb_rx_empty()) return false;

    uint8_t header[4];
    fusb_read_fifo(header, 4);
    pd->header = header[2] | (header[3] << 8);

    int count = PD_HEADER_NUM_DATA_OBJECTS(pd->header);
    if (count > 0)
        fusb_read_fifo((uint8_t*)pd->obj, count * 4);

    i2c_write_reg(FUSB302_REG_CONTROL1, FUSB302_CTL1_RX_FLUSH); // clear FIFO

    return true;
}

static void request_voltage(int mv, int ma){
    uint16_t hdr = 0;
    hdr |= (1<<6);    // Sink
    hdr |= (1<<5);    // PD Rev 3.0
    hdr |= (1<<4);    // Data = UFP
    hdr |= 1<<12;     // 1 Data Object

    uint32_t rdo = 0;
    rdo |= (0<<28);    // Object position = 1st PDO
    rdo |= ((ma/10)<<10);
    rdo |= (mv/50)<<0;

    uint8_t tx[20],i=0;
    tx[i++] = FUSB302_TX_TKN_SOP1;
    tx[i++] = FUSB302_TX_TKN_PACKSYM | 2; tx[i++]=hdr&0xff; tx[i++]=hdr>>8;
    tx[i++] = FUSB302_TX_TKN_PACKSYM | 4;
    memcpy(&tx[i],&rdo,4); i+=4;
    tx[i++] = FUSB302_TX_TKN_JAMCRC;
    tx[i++] = FUSB302_TX_TKN_EOP;
    tx[i++] = FUSB302_TX_TKN_TXOFF;
    tx[i++] = FUSB302_TX_TKN_TXON;

    uint8_t reg=FUSB302_REG_FIFOS;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, tx, i);
    fusb_write_reg(I2C1, FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_START);

    usart_printf("PD REQUEST sent: %dmV %dmA\n",mv,ma);
}

static void handle_pd_message(pd_msg_t *p){
    uint8_t type = PD_HEADER_MESSAGE_TYPE(p->header);

    if(type==1){ // Source Capabilities
        int cnt=PD_HEADER_NUM_DATA_OBJECTS(p->header);
        usart_printf("SOURCE_CAPS: %d PDOs\n",cnt);
        for(int i=0;i<cnt;i++){
            uint32_t obj=p->obj[i];
            int mv=((obj>>10)&0x3FF)*50;
            int ma=(obj&0x3FF)*10;
            printf(" PDO%d: %d mV  %d mA\n",i+1,mv,ma);
        }
        // request first profile automatically (5V normally)
        request_voltage(5000,3000);
    }
    else if(type==3) usart_printf(" ACCEPT\n");
    else if(type==6) usart_printf(" PS_RDY -> negotiation complete!\n");
}

/* ---- Interrupt Handling and Decoding Logic ---- */

void exti4_15_isr(void)
{
    usart_printf("EXTI handler triggered!\r\n");
    while (1) {
        if (exti_get_flag_status(EXTI8)) {
            /* USB-PD handling */
            pd_msg_t msg;
            if (read_pd_message(&msg))
                handle_pd_message(&msg);

            exti_reset_request(EXTI8);
        }
    }
}

/* ---- CLI parser ---- */
static void handle_command(char *line) {
    if (line[0] == 'r') {
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val;
        fusb_read_reg(I2C1, reg, &val);
        usart_printf("read[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 'w') {
        char *p = strtok(&line[1], " ");
        if (!p) { usart_printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t val = (uint8_t)strtol(p, NULL, 0);
        fusb_write_reg(I2C1, reg, val);
        usart_printf("write[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 'p') {
        usart_printf("Scanning I2C...\r\n");
        for (uint8_t addr=1; addr<0x7F; addr++) {
            if (i2c_probe_addr(I2C1, addr)) {
                usart_printf("Probed 0x%02X\r\n", addr);
            }
        }
    } else if (line[0] == 'b') {
        char *p = strtok(&line[1], " ");
        if (!p) { usart_printf("bulk read usage: b <reg>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        size_t nbytes = 80;
        uint8_t buf[80]; 
        fusb_read_reg_nbytes(I2C1, reg, buf, nbytes);
        usart_printf("read[0x%02X] = ", reg);
        for (int i=0; i<80; i++) {
            usart_printf("0x%02X ", buf[i]);
        }
    } else if (line[0] == 'n') {
        char *p = strtok(&line[1], " ");
        if (!p) { usart_printf("bulk write usage: n <reg> <val1> <val2> ...\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        uint8_t buf[40];
        size_t nbytes = 0;
        while ((p = strtok(NULL, " ")) != NULL && nbytes < sizeof(buf)) {
            buf[nbytes++] = (uint8_t)strtol(p, NULL, 0);
        }
        fusb_write_reg_nbytes(I2C1, reg, buf, nbytes);
        usart_printf("bulk wrote %u bytes to reg 0x%02X\r\n", (unsigned)nbytes, reg);
    } else if (line[0] == 't') {
        // Print bits set in register
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val;
        fusb_read_reg(I2C1, reg, &val);
        print_byte_as_bits(val, reg);
    } else if (line[0] == 's') {
        fusb_get_status();
    } else if (line[0] == 'c') {
        // Call a function by name
        char *p = strtok(&line[1], " ");
        if (!p) { usart_printf("usage: c <function_name>\r\n"); return; }
        if (strcmp(p, "fusb_measure_cc_pin_src") == 0) {
            int cc1_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC1);
            int cc2_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC2);
            usart_printf("CC1 level: %d, CC2 level: %d\r\n", cc1_lvl, cc2_lvl);
        } else if (strcmp(p, "fusb_check_cc_lines_src") == 0) {
            int ret = fusb_check_cc_pin_src();
            if (ret == 1) {
                usart_printf("Device detected on CC1.\r\n");
            } else if (ret == 2) {
                usart_printf("Device detected on CC2.\r\n");
            } else {
                usart_printf("No device detected on CC lines.\r\n");
            }
        } else if (strcmp(p, "fusb_measure_cc_pin_snk") == 0) {
            int cc1, cc2;
            fusb_measure_cc_pin_snk(&cc1, &cc2);
            usart_printf("---- CC Sink Measurements ----\r\n");
            usart_printf("CC1: 0x%02X, CC2: 0x%02X\r\n", cc1, cc2);
        } else if (strcmp(p, "fusb_get_chip_id") == 0) {
            uint8_t id = fusb_get_chip_id(I2C1);
            usart_printf("FUSB302 Chip ID (Reg: 0x01): 0x%02X\r\n", id);
        } else if (strcmp(p, "fusb_reset") == 0) {
            fusb_reset(I2C1);
            usart_printf("FUSB302 Reset (Reg: 0x0C) performed\r\n");
        } else if (strcmp(p, "fusb_power_all") == 0) {
            fusb_power_all(I2C1);
            usart_printf("FUSB302 Power (Reg: 0x0B) all on\r\n");
        } else if (strcmp(p, "fusb_pd_reset") == 0) {
            fusb_pd_reset(I2C1);
            usart_printf("FUSB302 PD Reset (Reg: 0x0C) performed\r\n");
        } else if (strcmp(p, "fusb_setup_sniffer") == 0) {
            fusb_setup_sniffer();
            usart_printf("FUSB302 Sniffer mode setup done\r\n");
        } else if (strcmp(p, "fusb_enable_gcrc") == 0) {
            p = strtok(NULL, " ");
            if (!p) { usart_printf("usage: c fusb_enable_gcrc <0|1>\r\n"); return; }
            bool enable = (strcmp(p, "1") == 0);
            fusb_enable_gcrc(I2C1, enable);
            usart_printf("FUSB302 AUTO_GCRC %s\r\n", enable ? "enabled" : "disabled");
        } else if (strcmp(p, "fusb_flush_rx") == 0) {
            fusb_flush_rx(I2C1);
            usart_printf("FUSB302 RX FIFO flushed\r\n");
        } else if (strcmp(p, "fusb_flush_tx") == 0) {
            fusb_flush_tx(I2C1);
            usart_printf("FUSB302 TX FIFO flushed\r\n");
        } else if (strcmp(p, "fusb_read_fifo") == 0) {
            uint8_t buf[32];
            fusb_read_fifo(buf, sizeof(buf));
            usart_printf("I2C Read FIFO: ");
            for (size_t i = 0; i < sizeof(buf); i++) {
                usart_printf("0x%02X ", buf[i]);
            }
            usart_printf("\r\n");
        } else if (strcmp(p, "check_rx_buffer") == 0) {
            check_rx_buffer();
        } else if (strcmp(p, "fusb_init_sink") == 0) {
            fusb_init_sink(I2C1);
        } else if (strcmp(p, "fusb_current_state") == 0) {
            fusb_current_state();
        } else if (strcmp(p, "fusb_setup") == 0) {
            fusb_setup();
        } else if (strcmp(p, "fusb_get_status") == 0) {
            fusb_get_status();
        } else if (strcmp(p, "fusb_check_cc_pin_snk") == 0) {
            fusb_check_cc_pin_snk();
        } else if (strcmp(p, "fusb_get_cc") == 0) {
            int cc1, cc2;
            fusb_get_cc(&cc1, &cc2);
            usart_printf("CC1: 0x%02X, CC2: 0x%02X\r\n", cc1, cc2);
        } else if (strcmp(p, "fusb_set_cc") == 0) {
            int pull = state.pulling_up;
            int ret = fusb_set_cc(pull);
            if (ret == 0) {
                usart_printf("Successfully set CC type.\r\n");
            } else {
                usart_printf("Failed to set CC type!\r\n");
            }
        } else {
            usart_printf("Unknown function: %s\r\n", p);
        }
    } else {
        usart_printf("Commands:\r\n  Read from register:\t\tr <reg>\r\n  Write to register:\t\tw <reg> <val>\r\n  Probe I2C addresses:\t\tp (probe)\r\n  Bulk read:\t\t\tb <reg>\r\n  Bulk write to register:\tn <reg> <val1> <val2> ...\r\n  Read bits in register:\tt <reg> \r\n  Status:\t\t\ts \r\n  Call function:\t\tc <name> \r\n");
    }
}

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
        // UART CLI Non-Blocking
        if (uart_rx_ready()) {
            char c = usart_recv(USART2);
            if (c=='\r' || c=='\n') {
                line[pos] = 0;
                usart_printf("\r\n");
                handle_command(line);
                pos = 0;
                usart_printf("> ");
            } 
            else if (pos < (int)sizeof(line)-1) {
                usart_send_blocking(USART2, c);  // echo character
                line[pos++] = c;
            }
        }

        // Event handling
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
