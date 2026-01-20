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
#include "pd.h"

/* ------------------------------------------------------------
 * Globals
 * ------------------------------------------------------------ */

//#define DEBUG_DUMP
#define I2C_TIMEOUT 100000

#define PACKET_IS_GOOD_CRC(head) (PD_HEADER_TYPE(head) == PD_CTRL_GOOD_CRC && PD_HEADER_CNT(head) == 0)
#define I2C_XFER_START BIT(0)
#define I2C_XFER_STOP BIT(1)

volatile uint32_t system_millis;

static struct fusb302_chip_state {
    // CC1 = 0, CC2 = 1
	int cc_polarity;
    // enabled = 1, disabled = 0
	int vconn_enabled;
	// pulling up (DFP) = 1, pulling down (UFP) = 0
	int pulling_up;
    // enable = 1, disable = 0
	int rx_enable;
	uint8_t mdac_vnc;
	uint8_t mdac_rd;
    // device attached = 1, no device attached = 0
    int attached;
    // 1 = sink sent power profile, 0 = no power profile sent
    int tx_sent;
} state;

// Struct to track PD communication state
static struct pd_protocol {
    // 3 bit rolling message id counter
    uint8_t msg_id;
    // power role (0 = sink, 1 = source)
    uint8_t power_role;
    // data role (0 = UFP, 1 = DFP)
    uint8_t data_role;
    // revision (0 = Rev 1.0, 1 = Rev 2.0, 2 = Rev 3.0)
    uint8_t rev;
} pd;

static struct pd_rx_messages {
    uint16_t head;
    uint32_t payload[7];
} rx_messages[50];

// counter for rx_messages index
int rx_messages_idx = 0;

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

static int i2c_start_write(uint32_t i2c, uint8_t addr, uint8_t nbytes, int flags)
{
    uint32_t timeout = 100000;
    uint32_t cr2 = 0;

    // Wait for bus free only if START requested
    if (flags & I2C_XFER_START) {
        while (I2C_ISR(i2c) & I2C_ISR_BUSY) {
            if (--timeout <= 0)
                return 1;
        }
    }

    cr2 |= (addr << 1);
    cr2 |= (nbytes << I2C_CR2_NBYTES_SHIFT);

    if (flags & I2C_XFER_STOP)
        cr2 |= I2C_CR2_AUTOEND;

    if (flags & I2C_XFER_START)
        cr2 |= I2C_CR2_START;

    I2C_CR2(i2c) = cr2;
    return 0;
}

static int i2c_start_read(uint32_t i2c, uint8_t addr, uint8_t nbytes, int flags)
{
    uint32_t cr2 = 0;

    cr2 |= (addr << 1);
    cr2 |= I2C_CR2_RD_WRN;
    cr2 |= (nbytes << I2C_CR2_NBYTES_SHIFT);

    if (flags & I2C_XFER_STOP)
        cr2 |= I2C_CR2_AUTOEND;

    if (flags & I2C_XFER_START)
        cr2 |= I2C_CR2_START;

    I2C_CR2(i2c) = cr2;
    return 0;
}

static int i2c_write_byte(uint32_t i2c, uint8_t val)
{
    uint32_t timeout = 100000;

    while (!(I2C_ISR(i2c) & I2C_ISR_TXIS)) {
        if (--timeout <= 0)
            return 1;
    }

    I2C_TXDR(i2c) = val;
    return 0;
}

static int i2c_read_byte(uint32_t i2c, uint8_t *val)
{
    uint32_t timeout = 100000;

    while (!(I2C_ISR(i2c) & I2C_ISR_RXNE)) {
        if (--timeout <= 0)
            return 1;
    }

    *val = I2C_RXDR(i2c);
    return 0;
}

static int fusb_xfer(const uint8_t *out, int out_size, uint8_t *in, int in_size, int flags)
{
    int i;

    // write phase
    if (out_size > 0) {
        if (i2c_start_write(I2C1, FUSB302_ADDR, out_size, flags))
            return 1;

        for (i = 0; i < out_size; i++)
            if (i2c_write_byte(I2C1, out[i]))
                return 1;
    }

    // read phase
    if (in_size > 0) {
        if (i2c_start_read(I2C1, FUSB302_ADDR, in_size, flags))
            return 1;

        for (i = 0; i < in_size; i++)
            if (i2c_read_byte(I2C1, &in[i]))
                return 1;
    }

    return 0;
}

static uint8_t fusb_rx_empty(void) 
{
    uint8_t status1 = fusb_read(FUSB302_REG_STATUS1);
    return (status1 & FUSB302_STATUS1_RX_EMPTY) != 0;
}

/* ------------------------------------------------------------
 * FUSB302 debug functions
 * ------------------------------------------------------------ */

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

static void fusb_check_control_regs(void)
{
    uint8_t reg;
    // Read and print control0 bits
    usart_printf("---- CONTROL0 ----\r\n");
    reg = fusb_read(FUSB302_REG_CONTROL0);
    dump_bits(reg, fusb302_control0_bits);
    usart_printf("\r\n");
    // Read and print control1 bits
    usart_printf("---- CONTROL1 ----\r\n");
    reg = fusb_read(FUSB302_REG_CONTROL1);
    dump_bits(reg, fusb302_control1_bits);
    usart_printf("\r\n");
    // Read and print control2 bits
    usart_printf("---- CONTROL2 ----\r\n");
    reg = fusb_read(FUSB302_REG_CONTROL2);
    dump_bits(reg, fusb302_control2_bits);
    usart_printf("\r\n");
    // Read and print control3 bits
    usart_printf("---- CONTROL3 ----\r\n");
    reg = fusb_read(FUSB302_REG_CONTROL3);
    dump_bits(reg, fusb302_control3_bits);
    usart_printf("\r\n");
}

static void fusb_check_switches_regs(void)
{
    uint8_t reg;
    // Read and print switches0 bits
    usart_printf("---- SWITCHES0 ----\r\n");
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    dump_bits(reg, fusb302_switches0_bits);
    usart_printf("\r\n");
    // Read and print switches1 bits
    usart_printf("---- SWITCHES1 ----\r\n");
    reg = fusb_read(FUSB302_REG_SWITCHES1);
    dump_bits(reg, fusb302_switches1_bits);
    usart_printf("\r\n");
}

static void fusb_check_mask_regs(void)
{
    uint8_t reg;
    // Read and print mask bits
    usart_printf("---- MASK ----\r\n");
    reg = fusb_read(FUSB302_REG_MASK);
    dump_bits(reg, fusb302_mask_bits);
    usart_printf("\r\n");
    // Read and print maskA bits
    usart_printf("---- MASKA ----\r\n");
    reg = fusb_read(FUSB302_REG_MASKA);
    dump_bits(reg, fusb302_maska_bits);
    usart_printf("\r\n");
    // Read and print maskB bits
    usart_printf("---- MASKB ----\r\n");
    reg = fusb_read(FUSB302_REG_MASKB);
    dump_bits(reg, fusb302_maskb_bits);
    usart_printf("\r\n");
}

static void check_rx_buffer(void)
{
    uint8_t buffer[80];
    fusb_read_fifo(buffer, 80);
    hexdump(buffer, 80);
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

/* ------------------------------------------------------------
 * FUSB302 functions
 * ------------------------------------------------------------ */

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

static void fusb_flush_tx_fifo(void)
{
    fusb_write(FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_FLUSH);
}

static void fusb_unmask_interrupts(void)
{
    // Unmask all interrupts
    fusb_write(FUSB302_REG_MASK, 0x00);
    fusb_write(FUSB302_REG_MASKA, 0x00);
    fusb_write(FUSB302_REG_MASKB, 0x00);
}

static void fusb_clear_interrupts()
{
    // Read interrupts to clear
    fusb_read(FUSB302_REG_INTERRUPT);
    fusb_read(FUSB302_REG_INTERRUPTA);
    fusb_read(FUSB302_REG_INTERRUPTB);
}

static void fusb_sop_prime_enable(bool enable)
{
    uint8_t reg;
    if (enable) {
        reg = fusb_read(FUSB302_REG_CONTROL1);
        reg |= (FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2);
    
    } else {
        reg = fusb_read(FUSB302_REG_CONTROL1);
        reg &= ~(FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2);
    }
    fusb_write(FUSB302_REG_CONTROL1, reg);
}

static void fusb_sop_prime_db_enable(bool enable)
{
    uint8_t reg;
    if (enable) {
        reg = fusb_read(FUSB302_REG_CONTROL1);
        reg |= (FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);

    } else {
        reg = fusb_read(FUSB302_REG_CONTROL1);
        reg &= ~(FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);
    }
    fusb_write(FUSB302_REG_CONTROL1, reg);
}

static void fusb_enable_gcrc(bool enable)
{
    uint8_t reg = fusb_read(FUSB302_REG_SWITCHES1);
    if (enable) {
        reg |= FUSB302_SW1_AUTO_GCRC;
    } else {
        reg &= ~FUSB302_SW1_AUTO_GCRC;
    }
    fusb_write(FUSB302_REG_SWITCHES1, reg);
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

static void fusb_send_hard_reset(void)
{
    fusb_flush_rx_fifo();
    uint8_t reg = fusb_read(FUSB302_REG_CONTROL3);
    reg |= FUSB302_CTL3_SEND_HARDRESET;
    fusb_write(FUSB302_REG_CONTROL3, reg);
    pd.msg_id = 0;
}

static void fusb_set_rp_default(void)
{
    uint8_t reg;
    reg = fusb_read(FUSB302_REG_CONTROL0);
    reg &= ~FUSB302_CTL0_HOST_CUR_MASK;
    reg |= FUSB302_CTL0_HOST_CUR_USB;
    state.mdac_vnc = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    state.mdac_rd = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_RD_MV);
    fusb_write(FUSB302_REG_CONTROL0, reg);
}

static void fusb_set_msg_header(uint8_t prole, uint8_t drole)
{
    uint8_t reg;
    reg = fusb_read(FUSB302_REG_SWITCHES1);
    reg &= ~(FUSB302_SW1_POWERROLE | FUSB302_SW1_DATAROLE);
    // Set power role
    if (prole)
        reg |= FUSB302_SW1_POWERROLE;
    // Set data role
    if (drole)
        reg |= FUSB302_SW1_DATAROLE;

    fusb_write(FUSB302_REG_SWITCHES1, reg);
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
    fusb_delay_us(350);
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
    fusb_delay_us(350);
    // Read cc1 measurement
    bc_lvl_cc1 = fusb_read(FUSB302_REG_STATUS0);
    // Read bc_lvl 
    bc_lvl_cc1 &= (FUSB302_STATUS0_BC_LVL0 | FUSB302_STATUS0_BC_LVL1);

    // Measure cc2
    reg = fusb_read(FUSB302_REG_SWITCHES0);
    // Disable cc1 measurement switch and enable cc2 measurement switch
    reg &= ~FUSB302_SW0_MEAS_CC1;
    reg |= FUSB302_SW0_MEAS_CC2;
    fusb_write(FUSB302_REG_SWITCHES0, reg);
    // Wait on measurement
    fusb_delay_us(350);
    // Read cc2 measurement
    bc_lvl_cc2 = fusb_read(FUSB302_REG_STATUS0);
    // Read bc_lvl
    bc_lvl_cc2 &= (FUSB302_STATUS0_BC_LVL0 | FUSB302_STATUS0_BC_LVL1);

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

static void fusb_set_cc(int pull)
{
    uint8_t reg;
    switch (pull) {
        case TYPEC_CC_RP:
            // enable correct pull-up
            reg = fusb_read(FUSB302_REG_SWITCHES0);
            reg &= ~(FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2 | FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2 | FUSB302_SW0_VCONN_CC1 | FUSB302_SW0_VCONN_CC2);
            reg |= (FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2);
            if (state.vconn_enabled) {
                // enable vconn on opposite cc line
                if (state.cc_polarity) {
                    reg |= FUSB302_SW0_VCONN_CC1;
                } else {
                    reg |= FUSB302_SW0_VCONN_CC2;
                }
            }
            fusb_write(FUSB302_REG_SWITCHES0, reg);
            state.pulling_up = 1;
            break;
        case TYPEC_CC_RD:
            // enable UFP
            // turn off toggle
            reg = fusb_read(FUSB302_REG_CONTROL2);
            reg &= ~FUSB302_CTL2_TOGGLE;
            fusb_write(FUSB302_REG_CONTROL2, reg);
            // enable pull-downs and disable pull-ups
            reg = fusb_read(FUSB302_REG_SWITCHES0);
            reg &= ~(FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2);
            reg |= (FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);
            fusb_write(FUSB302_REG_SWITCHES0, reg);
            state.pulling_up = 0;
            break;
        case TYPEC_CC_OPEN:
            // disable toggle
            reg = fusb_read(FUSB302_REG_CONTROL2);
            reg &= ~FUSB302_CTL2_TOGGLE;
            fusb_write(FUSB302_REG_CONTROL2, reg);
            // open manual switches
            reg = fusb_read(FUSB302_REG_SWITCHES0);
            reg &= ~(FUSB302_SW0_PU_EN1 | FUSB302_SW0_PU_EN2 | FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);
            fusb_write(FUSB302_REG_SWITCHES0, reg);
            state.pulling_up = 0;
            break;
        default:
            // unsupported
            break;
    }
}

static void fusb_set_polarity(int polarity)
{
    uint8_t reg;
    state.cc_polarity = polarity;

    reg = fusb_read(FUSB302_REG_SWITCHES0);

    // clear vconn bits
    reg &= ~(FUSB302_SW0_VCONN_CC1 | FUSB302_SW0_VCONN_CC2);
    if (state.vconn_enabled) {
        // enable vconn on opposite cc line
        if (polarity) {
            reg |= FUSB302_SW0_VCONN_CC1;
        } else {
            reg |= FUSB302_SW0_VCONN_CC2;
        }
    }
    // clear measure bits
    reg &= ~(FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);
    // set rx polarity
    if (polarity)
        reg |= FUSB302_SW0_MEAS_CC2;
    else
        reg |= FUSB302_SW0_MEAS_CC1;
    fusb_write(FUSB302_REG_SWITCHES0, reg);

    // clear tx cc bits
    reg = fusb_read(FUSB302_REG_SWITCHES1);
    reg &= ~(FUSB302_SW1_TXCC1 | FUSB302_SW1_TXCC2);
    // set tx polarity
    if (polarity)
        reg |= FUSB302_SW1_TXCC2;
    else
        reg |= FUSB302_SW1_TXCC1;
    fusb_write(FUSB302_REG_SWITCHES1, reg);
}

static void fusb_set_vconn(int enable)
{
    uint8_t reg;
    // update vconn state
    state.vconn_enabled = enable;
    
    if (enable) {
        // set saved polarity
        fusb_set_polarity(state.cc_polarity);
    } else {
        // clear both vconn_cc switches to disable vconn
        reg = fusb_read(FUSB302_REG_SWITCHES0);
        reg &= ~(FUSB302_SW0_VCONN_CC1 | FUSB302_SW0_VCONN_CC2);
        fusb_write(FUSB302_REG_SWITCHES0, reg);
    }
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
    // enable sop' and sop" reception
    fusb_sop_prime_enable(enable);
    fusb_sop_prime_db_enable(enable);

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

static int fusb_check_cc_voltage(void)
{
    bool cc1;
    if (state.cc_polarity)
        cc1 = false;
    else
        cc1 = true;
    int cc_mvolt = fusb_measure_cc_voltage(cc1);
    return cc_mvolt;
}

/* Parse header bytes for the size of packet */
static int get_num_bytes(uint16_t header)
{
	int rv;

	/* Grab the Number of Data Objects field.*/
	rv = PD_HEADER_CNT(header);

	/* Multiply by four to go from 32-bit words -> bytes */
	rv *= 4;

	/* Plus 2 for header */
	rv += 2;

	return rv;
}

static int fusb_get_message(uint32_t *payload, uint16_t *head)
{
    uint8_t buf[32];
    int rv;
    int len;

    if (fusb_rx_empty())
        return -1;

    do {
        buf[0] = FUSB302_REG_FIFOS;
         // Point FIFO read pointer (START, no STOP)
        rv = fusb_xfer(buf, 1, 0, 0, I2C_XFER_START);

        // Read token + PD header 3 bytes total (RESTART, no STOP)
        rv |= fusb_xfer(0, 0, buf, 3, I2C_XFER_START);

        // Parse PD header
        *head  = (buf[1] & 0xFF);
        *head |= ((buf[2] << 8) & 0xFF00);

        // Determine payload length (bytes, excluding header)
        len = get_num_bytes(*head) - 2;

        // Read payload + CRC (RESTART + STOP)
        rv |= fusb_xfer(0, 0, buf, len + 4, I2C_XFER_STOP);

    } while (!rv && PACKET_IS_GOOD_CRC(*head) && !fusb_rx_empty());

    // Exclude GoodCRC packets
    if (!PACKET_IS_GOOD_CRC(*head))
        // Payload excluding CRC
        memcpy(payload, buf, len);   

    return 0;
}

static int fusb_send_message(uint16_t header, const uint32_t *data, uint8_t *buf, int buf_pos)
{
    int rv;
    int reg;
    int len;

    len = get_num_bytes(header);

    // packsym tells the TXFIFO that the next X bytes are payload
    reg = FUSB302_TKN_PACKSYM;
    reg |= (len & 0x1F);
    buf[buf_pos++] = reg;

    // put in the header (2 bytes)
    reg = header;
    buf[buf_pos++] = reg & 0xFF;
    reg >>= 8;
    buf[buf_pos++] = reg & 0xFF;

    if (data == NULL) {
        // no data payload
        len = 0;
    } else {
        // data payload length in bytes
        len -= 2;
    }
    // put in the data payload
    memcpy(&buf[buf_pos], data, len);
    buf_pos += len;

    // put in the CRC
    buf[buf_pos++] = FUSB302_TKN_JAMCRC;

    // put in EOP
    buf[buf_pos++] = FUSB302_TKN_EOP;

    // Turn transmitter off after sending message
    buf[buf_pos++] = FUSB302_TKN_TXOFF;

    // Start transmission
    reg = FUSB302_TKN_TXON;
    buf[buf_pos++] = FUSB302_TKN_TXON;

    // burst write
    rv = fusb_xfer(buf, buf_pos, 0, 0, I2C_XFER_START | I2C_XFER_STOP);

    return rv;
}

/*
* @brief Transmit a Type-C message
* @param type: The type of Type-C message to send (SOP, SOP′, SOP″, Hard Reset, etc.)
* @param header: 16 bit header The PD header for the message
* @param data: Pointer to the data payload up to 7 × 32-bit PD Data Objects (or NULL for control messages)
*/
static int fusb_transmit(enum tcpc_message_type type, uint16_t header, const uint32_t *data)
{
	/*
     * Packet structure (40 bytes):
	 * 1: FIFO register address
	 * 4: SOP* tokens
	 * 1: Token that signifies "next X bytes are not tokens"
	 * 30: 2 for header and up to 7*4 = 28 for rest of message
	 * 1: "Insert CRC" Token
	 * 1: EOP Token
	 * 1: "Turn transmitter off" token
	 * 1: "Start Transmission" Command
	 */
    uint8_t buf[40];
    int buf_pos = 0;
    int reg;

    // Flush TX FIFO
    fusb_flush_tx_fifo();

    switch (type) {
        case TYPEC_MESSAGE_TYPE_SOP:
            // put fifo register at start
            buf[buf_pos++] = FUSB302_REG_FIFOS;
            // put ordered sop token into tx fifo
            buf[buf_pos++] = FUSB302_TKN_SOP1;
            buf[buf_pos++] = FUSB302_TKN_SOP1;
            buf[buf_pos++] = FUSB302_TKN_SOP1;
            buf[buf_pos++] = FUSB302_TKN_SOP2;
            return fusb_send_message(header, data, buf, buf_pos);
        case TYPEC_MESSAGE_TYPE_SOP_PRIME:
            // put fifo register at start
            buf[buf_pos++] = FUSB302_REG_FIFOS;
            // put ordered sop' token into tx fifo
            buf[buf_pos++] = FUSB302_TKN_SOP1;
            buf[buf_pos++] = FUSB302_TKN_SOP1;
            buf[buf_pos++] = FUSB302_TKN_SOP3;
            buf[buf_pos++] = FUSB302_TKN_SOP3;
            return fusb_send_message(header, data, buf, buf_pos);
        case TYPEC_MESSAGE_TYPE_SOP_DOUBLE_PRIME:
            // put fifo register at start
            buf[buf_pos++] = FUSB302_REG_FIFOS;
            // put ordered sop" token into tx fifo
            buf[buf_pos++] = FUSB302_TKN_SOP1;
            buf[buf_pos++] = FUSB302_TKN_SOP3;
            buf[buf_pos++] = FUSB302_TKN_SOP1;
            buf[buf_pos++] = FUSB302_TKN_SOP3;
            return fusb_send_message(header, data, buf, buf_pos);
        case TYPEC_MESSAGE_TYPE_HARD_RESET:
            // set send_hard_reset bit in control3
            reg = fusb_read(FUSB302_REG_CONTROL3);
            reg |= FUSB302_CTL3_SEND_HARDRESET;
            fusb_write(FUSB302_REG_CONTROL3, reg);
            break;
        case TYPEC_MESSAGE_TYPE_BIST_MODE_2:
            // set bist_mode_2 bit in control1
            reg = fusb_read(FUSB302_REG_CONTROL1);
            reg |= FUSB302_CTL1_BIST_MODE2;
            fusb_write(FUSB302_REG_CONTROL1, reg);
            // set start tx bit in control0
            reg = fusb_read(FUSB302_REG_CONTROL0);
            reg |= FUSB302_CTL0_TX_START;
            fusb_write(FUSB302_REG_CONTROL0, reg);
            fusb_delay_ms(50);
            // clear bist_mode_2 bit in control1
            reg = fusb_read(FUSB302_REG_CONTROL1);
            reg &= ~FUSB302_CTL1_BIST_MODE2;
            fusb_write(FUSB302_REG_CONTROL1, reg);
            break;
        default:
            return -1;
    }

    return 0;
}

// function to print status info for debugging
static void fusb_get_status(bool verbose)
{
    usart_printf("[%d] --- Status Update ---\r\n", system_millis);
    // log current fusb302 register state
    if (verbose) {
        fusb_check_status_regs();
        fusb_check_switches_regs();
        fusb_check_control_regs();
        fusb_check_mask_regs();
    }
    fusb_current_state();
    int vbus_voltage = fusb_measure_vbus_voltage();
    usart_printf("VBUS Voltage: %d mV\r\n", vbus_voltage);
    int cc_volt = fusb_check_cc_voltage();
    usart_printf("CC voltage: %d mV\r\n", cc_volt);
    usart_printf("[%d] --- End Status Update ---\r\n", system_millis);
}

static void fusb_setup(void)
{
    uint8_t reg;

    state.mdac_vnc = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    state.mdac_rd = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_RD_MV);

    // Reset FUSB302
    fusb_reset();

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

    // CONTROL0 mask interrupt
    reg = fusb_read(FUSB302_REG_CONTROL0);
    reg &= ~FUSB302_CTL0_INT_MASK;
    fusb_write(FUSB302_REG_CONTROL0, reg);

    // Set state defaults
    state.vconn_enabled = 0;
    state.cc_polarity = 0;
    state.attached = 0;
    state.tx_sent = 0;
    state.pulling_up = 0;

    // Power all
    fusb_power_all();
}

/* ------------------------------------------------------------
 * PD Helpers
 * ------------------------------------------------------------ */

static const char *pd_ctrl_msg_name(uint8_t type)
{
    switch (type) {
        case PD_CTRL_INVALID: return "Invalid";
        case PD_CTRL_GOOD_CRC: return "GoodCRC";
        case PD_CTRL_GOTO_MIN: return "GotoMin";
        case PD_CTRL_ACCEPT: return "Accept";
        case PD_CTRL_REJECT: return "Reject";
        case PD_CTRL_PING: return "Ping";
        case PD_CTRL_PS_RDY: return "PS_RDY";
        case PD_CTRL_GET_SOURCE_CAP: return "Get_Source_Cap";
        case PD_CTRL_GET_SINK_CAP: return "Get_Sink_Cap";
        case PD_CTRL_DR_SWAP: return "DR_Swap";
        case PD_CTRL_PR_SWAP: return "PR_Swap";
        case PD_CTRL_VCONN_SWAP: return "VCONN_Swap";
        case PD_CTRL_WAIT: return "Wait";
        case PD_CTRL_SOFT_RESET: return "Soft_Reset";
        /* Used for REV 3.0 */
        case PD_CTRL_DATA_RESET: return "Reset";
        case PD_CTRL_DATA_RESET_COMPLETE: return "Reset_Complete";
        case PD_CTRL_NOT_SUPPORTED: return "Not_Supported";
        case PD_CTRL_GET_SOURCE_CAP_EXT: return "Get_Source_Cap_Ext";
        case PD_CTRL_GET_STATUS: return "Get_Status";
        case PD_CTRL_FR_SWAP: return "FR_Swap";
        case PD_CTRL_GET_PPS_STATUS: return "Get_PPS_Status";
        case PD_CTRL_GET_COUNTRY_CODES: return "Get_Country_Code";
        case PD_CTRL_GET_SINK_CAP_EXT: return "Get_Sink_Cap_Ext";
        /* Used for REV 3.1 */
        case PD_CTRL_GET_SOURCE_INFO: return "Get_Source_Info";
        case PD_CTRL_GET_REVISION: return "Get_Revision";
        default:   return "Unknown_CTRL";
    }
}

static const char *pd_data_msg_name(uint8_t type)
{
    switch (type) {
        case PD_DATA_INVALID: return "Invalid";
        case PD_DATA_SOURCE_CAPABILITIES: return "Source_Capabilities";
        case PD_DATA_REQUEST: return "Request";
        case PD_DATA_BIST: return "BIST";
        case PD_DATA_SINK_CAPABILITIES: return "Sink_Capabilities";
        case PD_DATA_BATTERY_STATUS: return "Battery_Status";
        case PD_DATA_ALERT: return "Alert";
        case PD_DATA_GET_COUNTRY_INFO: return "Get_Country_Info";
        case PD_DATA_ENTER_USB: return "Enter_USB";
        case PD_DATA_EPR_REQUEST: return "EPR_Request";
        case PD_DATA_EPR_MODE: return "EPR_Mode";
        case PD_DATA_SOURCE_INFO: return "Source_Info";
        case PD_DATA_REVISION: return "Revision";
        case PD_DATA_VENDOR_DEFINED: return "Vendor_Defined";
        default:   return "Unknown_DATA";
    }
}

static void pd_log_header(uint16_t header)
{
    uint8_t cnt = PD_HEADER_CNT(header);
    uint8_t id = PD_HEADER_ID(header);
    uint8_t type = PD_HEADER_TYPE(header);
    uint8_t rev = PD_HEADER_REV(header);

    usart_printf("PD Rx Header: 0x%04X\r\n", header);
    usart_printf("\tMsgID: 0x%02X\r\n", id);
    usart_printf("\tRev: PD%d.0\r\n", rev+1);
    usart_printf("\tCount: %d\r\n", cnt);

    if (cnt > 0)
        usart_printf("\tType: %s\r\n", pd_data_msg_name(type));
    else
        usart_printf("\tType: %s\r\n", pd_ctrl_msg_name(type));
}

static void pd_log_source_caps(const uint32_t *pdo, int count)
{
    for (int i = 0; i < count; i++) {
        uint32_t p = pdo[i];

        uint8_t type = (p >> 30) & 0x3;

        if (type == 0) { // Fixed
            int mv = ((p >> 10) & 0x3FF) * 50;
            int ma = (p & 0x3FF) * 10;
            usart_printf("\tPDO%d: Fixed %dmV @ %dmA\r\n", i + 1, mv, ma);
        } else {
            usart_printf("\tPDO%d: Unsupported PDO type %u\r\n", i + 1, type);
        }
    }
}

/*
RDO Bit Mapping:
31-28 	Object Position: The index (1-7) of the PDO the sink is requesting.
27 		GiveBack: If 1, the sink will reduce power is source asks.
26		Capability Mismatch: If 1, the sink is requesting something source did not offer.
25		USB Comm Capable: If 1, the sink supports USB data communication.
24		No USB Suspend: If 1, the sink wants to stayed powered during USB suspend.
23		Unchunked Support: (PD 3.0) Supports large messages.
19-10	Operating Current: Current the sink will actually draw (10mA units).
9-0		Max Operating Current: Max current if GiveBack is 0 (10mA units)
*/
static void pd_log_request(uint32_t rdo) {
    uint8_t obj_pos = (rdo >> 28) & 0x07;
    uint32_t op_current = ((rdo >> 10) & 0x3FF) * 10; // 10mA units
    uint32_t max_current = (rdo & 0x3FF) * 10;        // 10mA units
    
    bool giveback = (rdo >> 27) & 0x01;
    bool mismatch = (rdo >> 26) & 0x01;
    bool usb_capable = (rdo >> 25) & 0x01;

    usart_printf("\tRequested PDO #%d\r\n", obj_pos);
    usart_printf("\tOperating Current: %dmA\r\n", op_current);
    
    if (giveback) {
        usart_printf("\tGiveBack Flag: SET (Min Current: %dmA)\r\n", max_current);
    } else {
        usart_printf("\tMax Current: %dmA\r\n", max_current);
    }

    if (mismatch) {
        usart_printf("\tCapability Mismatch Flag SET!!!\r\n");
    }

    if (usb_capable) {
        usart_printf("\tUSB data communication supported\r\n");
    } else {
        usart_printf("\tUSB data communication NOT supported\r\n");
    }
}

static void pd_check_rx_messages(void)
{
    uint16_t head;
    uint32_t payload[7];
    if (rx_messages_idx >= 50) {
        // reach limit, reset index
        rx_messages_idx = 0;
    }
    if (fusb_get_message(payload, &head) == 0) {
        rx_messages[rx_messages_idx].head = head;
        int hdr_cnt = PD_HEADER_CNT(head);
        for (int i = 0; i < hdr_cnt; i++) {
            rx_messages[rx_messages_idx].payload[i] = payload[i];
        }
        rx_messages_idx += 1;
    }
}

static void pd_dump_rx_messages(void)
{
    for (int i = 0; i <= rx_messages_idx; i++) {
        usart_printf("---- RX Message %d ----\r\n", i);
        int cnt = PD_HEADER_CNT(rx_messages[i].head);
        int type = PD_HEADER_TYPE(rx_messages[i].head);

        pd_log_header(rx_messages[i].head);
        if (cnt == 0) // control message
            continue;
        
        switch (type) {
            case 0x01:
                // source capabilities
                pd_log_source_caps(rx_messages[i].payload, cnt);
                break;
            case 0x02:
                // request
                pd_log_request(rx_messages[i].payload[0]);
                break;
            default:
                for (int j = 0; j < cnt; j++)
                    usart_printf("Payload[%d]: 0x%08X\r\n", j, rx_messages[i].payload[j]);
                break;
        }

        usart_printf("-----------------------\r\n");
    }
}

static void pd_init_src(void)
{
    pd.power_role = PD_POWER_ROLE_SOURCE;
    pd.data_role = PD_DATA_ROLE_DFP;
    pd.rev = PD_SPEC_REV2;
    pd.msg_id = 0;
    state.attached = 0;
    state.cc_polarity = 0;
    state.vconn_enabled = 0;
    state.pulling_up = 1;
    state.rx_enable = 0;
    state.tx_sent = 0;
    fusb_rx_enable(false);
    fusb_set_rp_default();
}

static void pd_init_snk(void)
{
    pd.power_role = PD_POWER_ROLE_SINK;
    pd.data_role = PD_DATA_ROLE_UFP;
    pd.rev = PD_SPEC_REV2;
    pd.msg_id = 0;
    state.attached = 0;
    state.cc_polarity = 0;
    state.vconn_enabled = 0;
    state.pulling_up = 0;
    state.rx_enable = 0;
    state.tx_sent = 0;
    fusb_rx_enable(false);
    fusb_set_rp_default();
}

static void pd_init(int pull)
{
    if (pull) {
        pd_init_src();
    } else {
        pd_init_snk();
    }
}

static void pd_send_src_caps(void)
{
    const uint32_t *src_pdo = pd_src_pdo;
    const int src_pdo_cnt = pd_src_pdo_cnt;
    uint16_t header;

    header = PD_HEADER(PD_DATA_SOURCE_CAPABILITIES, pd.power_role,
        pd.data_role, pd.msg_id, src_pdo_cnt,
        pd.rev, 0);
    fusb_transmit(TYPEC_MESSAGE_TYPE_SOP, header, src_pdo);
    usart_printf("Sent Source Capabilities with header: 0x%04X\r\n", header);
    pd.msg_id = (pd.msg_id + 1) & 0x07;
}

static void pd_send_snk_caps(void)
{
    const uint32_t *snk_pdo = pd_snk_pdo;
    const int snk_pdo_cnt = pd_snk_pdo_cnt;
    uint16_t header;

    header = PD_HEADER(PD_DATA_SINK_CAPABILITIES, pd.power_role,
        pd.data_role, pd.msg_id, snk_pdo_cnt,
        pd.rev, 0);
    fusb_transmit(TYPEC_MESSAGE_TYPE_SOP, header, snk_pdo);
    usart_printf("Sent Source Capabilities with header: 0x%04X\r\n", header);
    pd.msg_id = (pd.msg_id + 1) & 0x07;
}

static void pd_send_caps(void)
{
    if (!state.tx_sent) {
        if (state.pulling_up) {
            fusb_set_msg_header(PD_POWER_ROLE_SOURCE, PD_DATA_ROLE_DFP);
            pd_send_src_caps();
            state.tx_sent = 1;
        } else {
            fusb_set_msg_header(PD_POWER_ROLE_SINK, PD_DATA_ROLE_UFP);
            pd_send_snk_caps();
            state.tx_sent = 1;
        }
    }
}

/* ------------------------------------------------------------
 * Interrupt handling
 * ------------------------------------------------------------ */

// Handler for EXTI4_15_IRQ (handles PB8 interrupt)
void exti4_15_isr(void) {
    if (exti_get_flag_status(EXTI8)) {
        // Read the Interrupt registers to see what happened and clear the interrupt.
        uint8_t int_a = fusb_read(FUSB302_REG_INTERRUPTA);
        uint8_t int_b = fusb_read(FUSB302_REG_INTERRUPTB);
        uint8_t int_c = fusb_read(FUSB302_REG_INTERRUPT);

        // Hard reset ordered set received
        if (int_a & FUSB302_INTA_HARDRST) {
            usart_printf("INT: Hard Reset Received.\r\n");
            // Hard reset requires clearing state and re-initializing fusb302
            fusb_reset();
            fusb_setup();
            pd_init(state.pulling_up);
        }

        // Received a soft reset packet
        if (int_a & FUSB302_INTA_SOFTRST) {
            usart_printf("INT: Soft Reset Received.\r\n");
            // Soft reset
            fusb_pd_reset();
        }

        // A received message is confirmed when GoodCRC is sent (GCRCSENT)
        if (int_b & FUSB302_INTB_GCRCSENT) {
            usart_printf("INT: GoodCRC Sent (Packet Received)\r\n");
            if (int_c & FUSB302_INT_CRC_CHK) {
#ifdef DEBUG_DUMP
                check_rx_buffer();
#else
                // I_CRC_CHK bit in INTERRUPT register indicates a received PD message
                pd_check_rx_messages();
                pd_dump_rx_messages();
#endif
            }
        }
        
        // CC comparator change
        if (int_c & FUSB302_INT_COMP_CHNG) {
            uint8_t status0 = fusb_read(FUSB302_REG_STATUS0);
            uint8_t bc_lvl = (status0 & FUSB302_STATUS0_BC_LVL_MASK) >> FUSB302_STATUS0_BC_LVL_POS;
            usart_printf("INT: CC line tripped threshold programmed into MDAC (BC_LVL=0x%02X)\r\n", bc_lvl);
        }

        // Clear interrupts
        fusb_clear_interrupts();
        // Clear the EXTI
        exti_reset_request(EXTI8); 
    }
}

/* ------------------------------------------------------------
 * Main Program Functions
 * ------------------------------------------------------------ */

// poll function to get/set changes in state
static void poll(void)
{
    int attached = fusb_int_vbusok();
    if (attached != state.attached) {
        if (attached) {
            state.attached = 1;
            usart_printf("[%d] - Attach detected\r\n", system_millis);
            int polarity = fusb_check_cc_pin();
            int cc_n = polarity ? 2 : 1;
            usart_printf("[%d] - CC line on CC%d\r\n", system_millis, cc_n);
            fusb_set_cc(state.pulling_up ? TYPEC_CC_RP : TYPEC_CC_RD);
            fusb_set_polarity(polarity);
            fusb_set_msg_header(pd.power_role, pd.data_role);
            fusb_set_vconn(state.vconn_enabled);
            fusb_rx_enable(true);
            fusb_get_status(false);
        } else {
            // verify device is dettached
            int still_attached = fusb_check_cc_voltage();
            // if CC voltage is 0, assume device is not attached
            if (!still_attached) {
                usart_printf("[%d] - Dettach detected\r\n", system_millis);
                // set default state
                pd_init(0);
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
        fusb_get_status(true);
        check_rx_buffer();
    } else if (line[0] == 'c') {
        pd_dump_rx_messages();
    } else if (line[0] == 'x') {
        char *p = strtok(&line[1], " ");
        if (!p) { usart_printf("usage: x <type> <prole> <drole> <id> <cnt> <rev> <ext>\r\n"); return 0; }
        uint8_t type = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: x <type> <prole> <drole> <id> <cnt> <rev> <ext>\r\n"); return 0; }
        uint8_t prole = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: x <type> <prole> <drole> <id> <cnt> <rev> <ext>\r\n"); return 0; }
        uint8_t drole = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: x <type> <prole> <drole> <id> <cnt> <rev> <ext>\r\n"); return 0; }
        uint8_t id = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: x <type> <prole> <drole> <id> <cnt> <rev> <ext>\r\n"); return 0; }
        uint8_t cnt = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: x <type> <prole> <drole> <id> <cnt> <rev> <ext>\r\n"); return 0; }
        uint8_t rev = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { usart_printf("usage: x <type> <prole> <drole> <id> <cnt> <rev> <ext>\r\n"); return 0; }
        uint8_t ext = (uint8_t)strtol(p, NULL, 0);
        uint16_t header = PD_HEADER(type, prole, drole, id, cnt, rev, ext);
        usart_printf("Sending message with header: 0x%04X\r\n", header);
        fusb_transmit(TYPEC_MESSAGE_TYPE_SOP, header, NULL);
    } else if (line[0] == 'b') {
        check_rx_buffer();
    } else if (line[0] == 'p') {
        pd_send_caps();
    } else if (line[0] == 'q') {
        // return 1 to tell debug_cli to break loop and return to logging
        return 1;
    } else {
        usart_printf("Commands:\r\n  Read from register:\t\tr <reg>\r\n  Write to register:\t\tw <reg> <val>\r\n  Read bits in register:\tt <reg> \r\n  Status:\t\t\ts \r\n  Check Rx messages:\t\tc  \r\n  Send SOP message:\t\tx <type> <prole> <drole> <id> <cnt> <rev> <ext>  \r\n  Read FIFO:\t\t\tb  \r\n  Send Capabilities:\t\tp  \r\n  Quit:\t\t\t\tq  \r\n");
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
    // initialize as sink
    pd_init(0);

    while (1) {
        if (usart_rx_ready()) {
            char c = usart_recv(USART2);
            if (c=='\r' || c=='\n') {
                usart_printf("Logging paused. Entering debug menu...\r\n");
                debug_cli();
            } 
        }
        poll();

        // small delay to avoid busy looping
        fusb_delay_ms(700);
    }

}

