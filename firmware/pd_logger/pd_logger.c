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

static struct fusb302_chip_state {
    // CC1 = 0, CC2 = 1
	int cc_polarity;
	int vconn_enabled;
	// pulling up (DFP) = 1, pulling down (UFP) = 0
	int pulling_up;
    // enable = 1, disable = 0
	int rx_enable;
	uint8_t mdac_vnc;
	uint8_t mdac_rd;
    // device attached = 1, no device attached = 0
    int attached;
} state;

/* ------------------------------------------------------------
 * MCU Setup Functions
 * ------------------------------------------------------------ */

void sys_tick_handler(void)
{
    system_millis++;
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_I2C1);
    // We need to enable the clock for SYSCFG to configure EXTI.
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);
}

static void usart_setup(void)
{
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

static void i2c_setup(void)
{
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

static void exti_setup(void)
{
    // Map PB8 to EXTI8
    exti_select_source(EXTI8, GPIOB);

    // Set EXTI8 to trigger on a falling edge (INT_N is active-low)
    exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);

    // Enable EXTI8 interrupt line
    exti_enable_request(EXTI8);

    /* Source Identification: Inside the EXTI4_15_IRQHandler function, we will 
    need to check the specific pending register flag (PR register, bit 8) for EXTI 
    line 8 to determine if it was the source of the interrupt, as lines 4 through 15 
    all share this single handler */
    nvic_enable_irq(NVIC_EXTI4_15_IRQ); 
}

/* ------------------------------------------------------------
 * USART logging functions
 * ------------------------------------------------------------ */

/* simple blocking getchar/putchar */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}

static void usart_send_char(char c)
{
    usart_send_blocking(USART2, c);
}

static char usart_getc(void)
{ 
    return usart_recv_blocking(USART2); 
}

static void usart_printf(const char *format, ...)
{
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

static void print_byte_as_bits(uint8_t byte, uint8_t reg)
{
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

static void hexdump(const uint8_t *data, size_t len)
{
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

static bool usart_rx_ready(void)
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

static uint8_t fusb_read(uint8_t reg)
{
    uint8_t v;
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, &v, 1);
    return v;
}

static void fusb_read_fifo(uint8_t *buf, int len)
{
    uint8_t r = FUSB302_REG_FIFOS;
    i2c_transfer7(I2C1, FUSB302_ADDR, &r, 1, buf, len);
}

/* ------------------------------------------------------------
 * FUSB302 functions
 * ------------------------------------------------------------ */

static bool fusb_rx_empty(void) 
{
    return fusb_read(FUSB302_REG_STATUS1) & FUSB302_STATUS1_RX_EMPTY;
}

static void fusb_delay_ms(uint32_t ms)
{
    // This assumes SysTick is running at 1ms intervals.
    for (uint32_t i = 0; i < ms; i++) {
        // Wait for the SysTick flag to be set (1ms elapsed)
        while ((STK_CSR & STK_CSR_COUNTFLAG) == 0);
    }
}

static void fusb_delay_us(uint32_t us) {
    // At 48MHz, approximately 48 clock cycles per microsecond
    // Each NOP takes 1 cycle, loop overhead is minimal
    for (uint32_t i = 0; i < us * 6; i++) {
        __asm__("nop");
    }
}

static void fusb_reset(void)
{
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(2);
}

static void fusb_pd_reset(void)
{
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_PD);
    fusb_delay_ms(2);
}

static void fusb_power_all(void)
{
    fusb_write(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(1);
}

static void fusb_flush_rx_fifo(void)
{
    fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_RX_FLUSH);
}

static void fusb_flush_Tx_fifo(void)
{
    fusb_write(FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_FLUSH);
}

static void fusb_unmask_all_interrupts(void)
{
    // Unmask all interrupts
    fusb_write(FUSB302_REG_MASK, 0x00);
    fusb_write(FUSB302_REG_MASKA, 0x00);
    fusb_write(FUSB302_REG_MASKB, 0x00);
    // Reading interrupts clears them
    fusb_read(FUSB302_REG_INTERRUPT);
    fusb_read(FUSB302_REG_INTERRUPTA);
    fusb_read(FUSB302_REG_INTERRUPTB);
}

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
    if (state.attached) {
        usart_printf("Device Attached: True\r\n");
    } else {
        usart_printf("Device Attached: False\r\n");
    }
    usart_printf("---- End State ----\r\n");
}

static void fusb_init_sink(void)
{
    // CONTROL1 Enable reception of all SOP packets
    fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);

    // SWITCHES1 Enable Auto-CRC, Set sink role
    fusb_write(FUSB302_REG_SWITCHES1, FUSB302_SW1_AUTO_GCRC | FUSB302_SW1_SPECREV1 | FUSB302_SW1_SPECREV0);

    // CONTROL2 Toggle to detect CC and establish UFP (Sink)
    fusb_write(FUSB302_REG_CONTROL2, FUSB302_CTL2_MODE_UFP | FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE);

    state.pulling_up = 0; // Sink mode, pulling down
    
    usart_printf("FUSB302 initialized in Sink mode.\n");
}

static void fusb_setup(void)
{
    uint8_t reg;

    state.mdac_vnc = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    state.mdac_rd = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_RD_MV);

    // Reset FUSB302
    fusb_reset();

    // Power all
    fusb_power_all();

    // CONTROL3 Turn on retries and set number of retries
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
    
    // MaskB GoodCRC to ack pd message
    reg = 0xFF;
    reg &= ~FUSB302_MASKB_GCRCSENT;
    fusb_write(FUSB302_REG_MASKB, reg);

    // CONTROL0 Enable interrupt
    reg = fusb_read(FUSB302_REG_CONTROL0);
    reg &= ~FUSB302_CTL0_INT_MASK;
    fusb_write(FUSB302_REG_CONTROL0, reg);

    // Set state defaults
    state.vconn_enabled = 0;
    state.cc_polarity = 0;
    state.attached = 0;
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
    uint8_t buffer[80];
    // dump fifo if not empty
    if (!fusb_rx_empty()) {
        fusb_read_fifo(buffer, 80);
        hexdump(buffer, 80);
    }
}

static int convert_bc_lvl(int bc_lvl)
{
    int tc_lvl = TYPEC_CC_VOLT_OPEN;
    if (state.pulling_up) {
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
    uint8_t mdac = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV); // converts milivolts to MDAC code to write to MEASURE register
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

static void fusb_measure_cc_pin_snk(uint8_t *cc1, uint8_t *cc2)
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
    // Read bc_lvl 
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
    // Read bc_lvl
    bc_lvl_cc2 &= (FUSB302_STATUS0_BC_LVL0 | FUSB302_STATUS0_BC_LVL1);
    usart_printf("CC2 Sink BC_LVL: %02X\r\n", bc_lvl_cc2);

    *cc1 = convert_bc_lvl(bc_lvl_cc1);
    *cc2 = convert_bc_lvl(bc_lvl_cc2);

    // Restore switches0 to orignal state
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

static void fusb_enable_gcrc(bool enable)
{
    // AUTO_GCRC is in SWITCHES1 register
    uint8_t reg = fusb_read(FUSB302_REG_SWITCHES1);
    if (enable) {
        reg |= FUSB302_SW1_AUTO_GCRC;
    } else {
        reg &= ~FUSB302_SW1_AUTO_GCRC;
    }
}

static int fusb_check_cc_pin_snk(void)
{
    // return 0 for cc1, 1 for cc2
    int ret = 0; // default cc1
    uint8_t cc1, cc2;
    fusb_measure_cc_pin_snk(&cc1, &cc2);
    if (cc1 > cc2) {
        ret = 0;
        state.cc_polarity = 0;
    } else {
        ret = 1;
        state.cc_polarity = 1;
    }
    return ret;
}

static int fusb_check_cc_pin(void)
{
    // return 0 for cc1, 1 for cc2
    int ret = 0; // default cc1
    if (state.pulling_up) {
        // measure cc line for source
        int cc1_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC1);
        int cc2_lvl = fusb_measure_cc_pin_src(FUSB302_SW0_MEAS_CC2);
        if (cc1_lvl != TYPEC_CC_VOLT_OPEN && cc2_lvl == TYPEC_CC_VOLT_OPEN) {
            ret = 0; // Device detected on CC1
            state.cc_polarity = 0;
        } else if (cc2_lvl != TYPEC_CC_VOLT_OPEN && cc1_lvl == TYPEC_CC_VOLT_OPEN) {
            ret = 1; // Device detected on CC2
            state.cc_polarity = 1;
        }
    } else {
        ret = fusb_check_cc_pin_snk();
    }
    return ret;
}

// enable rx must be called after an attach and cc polarity set
static void fusb_rx_enable(bool enable)
{
    uint8_t reg;
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    // clear measure bits
    reg &= ~(FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);
    if (enable) {
        state.rx_enable = 1;
        if (state.cc_polarity) {
            // set cc2 measure bit
            reg |= FUSB302_SW0_MEAS_CC2;
        } else {
            // set cc1 measure bit
            reg |= FUSB302_SW0_MEAS_CC1;
        }
        // write cc measure to switches0 register
        fusb_write(FUSB302_REG_SWITCHES0, reg);
        // mask bc_lvl interrupt
        reg = fusb_read(FUSB302_REG_MASK);
        reg |= FUSB302_MASK_BC_LVL;
        fusb_write(FUSB302_REG_MASK, reg);
        // flush rx fifo
        fusb_flush_rx_fifo();
    } else {
        state.rx_enable = 0;
        // disable cc_meas
        fusb_write(FUSB302_REG_SWITCHES0, reg);
        // enable bc_lvl interrupt
        reg = fusb_read(FUSB302_REG_MASK);
        reg &= ~FUSB302_MASK_BC_LVL;
        fusb_write(FUSB302_REG_MASK, reg);
    }
    fusb_enable_gcrc(enable);
}

static int fusb_mdac_comp(int mdac)
{
    int orig_reg, reg;
    // save original REG_MEASURE register
    orig_reg = fusb_read(FUSB302_REG_MEASURE);
    // set mdac to reg_measure bits 0-5 and set bit 6 to measure vbus
    fusb_write(FUSB302_REG_MEASURE, (mdac & FUSB302_MEAS_MDAC_MASK) | FUSB302_MEAS_VBUS);
    fusb_delay_us(350);
    // Read status0 register, if STATUS0_COMP=1 then vbus is higher than (mdac + 1) * 0.42V
    reg = fusb_read(FUSB302_REG_STATUS0);
    // restore original value
    fusb_write(FUSB302_REG_MEASURE, orig_reg);
    return reg & FUSB302_STATUS0_COMP;
}

// Returns voltage on VBUS in mV
static int fusb_measure_vbus_voltage(void)
{
    int vbus; 
    int mdac = 0;
    // compare vbus with mdac voltage
    // check each bit of reg_measure [5:0]
    for (int i = 5; i >= 0; i--) {
        if (fusb_mdac_comp(mdac | (1 << i))) {
            mdac |= (1 << i);
        }
    }
    vbus = (mdac + 1) * 420;
    return vbus;
}

static int fusb_measure_cc_voltage(bool cc1)
{
    uint8_t sw0 = fusb_read(FUSB302_REG_SWITCHES0);
    uint8_t meas = fusb_read(FUSB302_REG_MEASURE);

    // Disable Rd and select CC pin
    uint8_t new_sw0 = sw0 & ~(FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);
    new_sw0 &= ~(FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);
    new_sw0 |= cc1 ? FUSB302_SW0_MEAS_CC1 : FUSB302_SW0_MEAS_CC2;

    fusb_write(FUSB302_REG_SWITCHES0, new_sw0);

    uint8_t dac;
    for (dac = 0; dac < 64; dac++) {
        fusb_write(FUSB302_REG_MEASURE, (meas & ~FUSB302_MEAS_MDAC_MASK) | dac);

        // comparator settle
        fusb_delay_us(350);

        uint8_t st = fusb_read(FUSB302_REG_STATUS0);

        // COMP = 1 → CC > DAC 
        if (!(st & FUSB302_STATUS0_COMP))
            break;
    }

    // Restore registers
    fusb_write(FUSB302_REG_MEASURE, meas);
    fusb_write(FUSB302_REG_SWITCHES0, sw0);

    // Approximate conversion to mV
    return dac * 42;
}

static int fusb_int_vbusok(void)
{
    // Note: interrupt is cleared when read
    // return 1 for vbusok else 0
    uint8_t reg = fusb_read(FUSB302_REG_INTERRUPT);
    if (reg & FUSB302_INT_VBUSOK) {
        return 1;
    } else {
        return 0;
    }
}

static int fusb_check_cc_voltage(void)
{
    bool cc1;
    int polarity = fusb_check_cc_pin();
    if (polarity)
        cc1 = false;
    else
        cc1 = true;
    int cc_mvolt = fusb_measure_cc_voltage(cc1);
    return cc_mvolt;
}

// function to print status info for debugging
static void fusb_get_status(void)
{
    fusb_check_status_regs();
    usart_printf("INT pin=%02X\r\n", gpio_get(GPIOB, GPIO8) ? 1 : 0);
    fusb_current_state();
    if (!fusb_rx_empty()) {
        check_rx_buffer();
    }
    int vbus_voltage = fusb_measure_vbus_voltage();
    usart_printf("VBUS Voltage: %d mV\r\n", vbus_voltage);
    int cc_volt = fusb_check_cc_voltage();
    usart_printf("CC voltage: %d mV\r\n", cc_volt);
}

/* ------------------------------------------------------------
 * PD Message and Interrupt Handling
 * ------------------------------------------------------------ */

// static uint8_t rx_buffer[MAX_PD_PACKET_SIZE];
// void exti4_15_isr(void) {
//     usart_printf("EXTI interrupt triggered!\r\n");
//     // Check if the interrupt is from EXTI line 8 (PB8)
//     if (exti_get_flag_status(EXTI8)) {
//         usart_printf("INT_N pin asserted!\r\n");
//         // Reading Interrupt register clears the interrupt condition
//         uint8_t irq_status = fusb_read(FUSB302_REG_INTERRUPT); 
//         uint8_t status1 = fusb_read(FUSB302_REG_STATUS1); // Read status register
        
//         if (irq_status & FUSB302_INT_CRC_CHK) { // Valid packet received
            
//             if (status1 & FUSB302_STATUS1_RX_EMPTY) {
//                 // Should not happen if CRC_CHK is set, but check anyway
//                 usart_printf("IRQ triggered but FIFO empty.\r\n");
//                 goto end_irq;
//             }

//             // Read SOP Token
//             // SOP token is the first byte in FIFO
//             uint8_t sop_token = fusb_read(FUSB302_REG_FIFOS);
//             usart_printf("SOP Token: %02X\r\n", sop_token);
//             rx_buffer[0] = sop_token;
//             size_t bytes_read = 1;
            
//             // Read Message Header (2 bytes)
//             fusb_read_fifo(&rx_buffer[bytes_read], 2);
//             uint16_t header = (uint16_t)rx_buffer[bytes_read] | (uint16_t)(rx_buffer[bytes_read + 1] << 8);
//             usart_printf("Message header: %04X\r\n", header);
//             bytes_read += 2;
            
//             size_t payload_words = 0;
//             size_t total_data_bytes = 0;
//             const char* packet_type_str = "CTRL";

//             if (PD_HEADER_EXTENDED(header)) {
//                 // Extended Message
//                 packet_type_str = "EXTD";
                
//                 // Read Extended Message Header (2 bytes)
//                 fusb_read_fifo(&rx_buffer[bytes_read], 2);
//                 uint16_t ext_header = (uint16_t)rx_buffer[bytes_read] | (uint16_t)(rx_buffer[bytes_read + 1] << 8);
//                 usart_printf("Extended message header: %04X\r\n", ext_header);
//                 bytes_read += 2;
                
//                 // Data Size (B8:0) tells total payload size in bytes
//                 uint16_t data_size = ext_header & 0x01FF;
//                 usart_printf("Data size: %04X\r\n", data_size);
//                 total_data_bytes = data_size;
                
//                 // The actual payload data might be padded. For sniffing, read payload bytes.
//                 // Payload must be read as multiple of 4 bytes (Data Objects) 
//                 // up to MaxExtendedMsgLegacyLen (26 bytes, if chunking is involved)
//                 // For simplicity, we assume reading the full length specified by Data Size + CRC (4 bytes)
//                 size_t remaining_bytes_to_read = total_data_bytes + 4; // Data + CRC
                
//                 // Safely clamp remaining read length to avoid buffer overflow
//                 if (bytes_read + remaining_bytes_to_read > MAX_PD_PACKET_SIZE) {
//                     remaining_bytes_to_read = MAX_PD_PACKET_SIZE - bytes_read;
//                 }
                
//                 fusb_read_fifo(&rx_buffer[bytes_read], remaining_bytes_to_read);
//                 bytes_read += remaining_bytes_to_read;
//                 check_rx_buffer();
                
//             } else {
//                 // Control or Data Message
//                 payload_words = PD_HEADER_NUM_DATA_OBJECTS(header);
//                 total_data_bytes = payload_words * 4; // 1 Data Object = 4 bytes
                
//                 if (payload_words > 0) {
//                     packet_type_str = "DATA";
//                 }
                
//                 size_t total_payload_plus_crc = total_data_bytes + 4; // CRC is 4 bytes
//                 size_t remaining_bytes_to_read = total_payload_plus_crc;

//                 // Safely clamp remaining read length
//                 if (bytes_read + remaining_bytes_to_read > MAX_PD_PACKET_SIZE) {
//                     remaining_bytes_to_read = MAX_PD_PACKET_SIZE - bytes_read;
//                 }
                
//                 fusb_read_fifo(&rx_buffer[bytes_read], remaining_bytes_to_read);
//                 bytes_read += remaining_bytes_to_read;
//                 check_rx_buffer();
//             }
            
//             // Log Output
//             usart_printf("--- PD PACKET SNIFFED ---\r\n");
//             usart_printf("Type: %s (DOs: %d) | Total Bytes: %d\r\n", packet_type_str, (int)payload_words, (int)bytes_read);
            
//             // Print raw buffer content
//             for (size_t i = 0; i < bytes_read; i++) {
//                 usart_printf("%02X ", rx_buffer[i]);
//                 if (i % 16 == 15) usart_printf("\r\n");
//             }
//             usart_printf("\r\n-------------------------\r\n");
            
//             // Flush RxFIFO completely if necessary
//             // fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_RX_FLUSH); 
            
//         } else if (irq_status & FUSB302_INT_ACTIVITY) {
//             // Log CC activity if CRC_CHK wasn't set, might indicate noise or invalid frame
//             usart_printf("CC Activity Detected (Non-CRC event).\r\n");
//         }

//         end_irq:
//         // Clear the pending EXTI interrupt flag
//         exti_reset_request(EXTI8);
//     }
// }

/* ------------------------------------------------------------
 * Main Program Functions
 * ------------------------------------------------------------ */

// poll function to get/set changes in state
static void poll(void)
{
    int attached = fusb_int_vbusok();
    int cc_volt = fusb_check_cc_voltage();
    if (!attached && cc_volt > 0)
        // account for case where VBUS is pulled to ground with jumper
        attached = 1;
    if (attached != state.attached) {
        if (attached) {
            state.attached = 1;
            usart_printf("Attach detected: 0x%02X\r\n", attached);
            usart_printf("Detecting CC pin...\r\n");
            int polarity = fusb_check_cc_pin();
            int cc_n = polarity ? 2 : 1;
            usart_printf("CC line on CC%d\r\n", cc_n);
            fusb_get_status();
            // enable rx
            fusb_rx_enable(true);
            // check rx fifo. TODO: replace with PD message handler.
            check_rx_buffer();
        } else {
            // reading interrupts clears them, so we need a work around to avoid false positives
            // verify device is dettached
            int still_attached = fusb_check_cc_voltage();
            // if CC voltage is 0, assume device is not attached (some edge cases will be missed)
            if (!still_attached) {
                usart_printf("Dettach detected\r\n");
                // disable rx
                fusb_rx_enable(false);
                // set default state
                state.attached = 0;
                state.cc_polarity = 0;
                state.vconn_enabled = 0;
                fusb_pd_reset();
            }
        }
    }
}

// CLI parser
static int handle_command(char *line) {
    if (line[0] == 'r') {
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val = fusb_read(reg);
        usart_printf("read[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 'w') {
        char *p = strtok(&line[1], " ");
        if (!p) { usart_printf("usage: w <reg> <val>\r\n"); return 0; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: w <reg> <val>\r\n"); return 0; }
        uint8_t val = (uint8_t)strtol(p, NULL, 0);
        fusb_write(reg, val);
        usart_printf("write[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 't') {
        // Print bits set in register
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val = fusb_read(reg);
        print_byte_as_bits(val, reg);
    } else if (line[0] == 's') {
        fusb_get_status();
        check_rx_buffer();
    } else if (line[0] == 'q') {
        // return 1 to tell debug_cli to break loop and return to logging
        return 1;
    } else {
        usart_printf("Commands:\r\n  Read from register:\t\tr <reg>\r\n  Write to register:\t\tw <reg> <val>\r\n  Read bits in register:\tt <reg> \r\n  Status:\t\t\ts \r\n  Quit:\t\t\t\tq  \r\n");
    }
    return 0;
}

static void debug_cli(void)
{
    usart_printf("---- PD Logger Debug Console ----\r\n");
    char line[32];
    int pos = 0;

    while (1) {
        // UART CLI Non-Blocking
        if (usart_rx_ready()) {
            char c = usart_recv(USART2);
            if (c=='\r' || c=='\n') {
                line[pos] = 0;
                usart_printf("\r\n");
                int break_signal = handle_command(line);
                if (break_signal) {
                    usart_printf("Returning to live logging...\r\n");
                    break;
                }
                pos = 0;
                usart_printf("> ");
            } 
            else if (pos < (int)sizeof(line)-1) {
                usart_send_blocking(USART2, c);  // echo character
                line[pos++] = c;
            }
        }
    }
}

int main(void)
{
    clock_setup();
    systick_setup();
    usart_setup();
    i2c_setup();
    exti_setup(); 

    fusb_setup();

    while (1) {
        if (usart_rx_ready()) {
            char c = usart_recv(USART2);
            if (c=='\r' || c=='\n') {
                usart_printf("Logging paused. Entering debug menu...\r\n");
                debug_cli();
            } 
        }
        poll();
        fusb_delay_ms(1000);
    }
}