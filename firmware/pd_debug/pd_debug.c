#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
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

    /* Hardware reset via RCC */
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);

    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

/*---- Helper/Essential Functions ----*/

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

static void print_byte_as_bits(uint8_t byte, uint8_t reg) {
    printf("Reg %02X: ", reg);
    for (int i = 7; i >= 0; i--) { // Loop from most significant bit (7) to least significant (0)
        // Use a bitwise AND with a mask to check if the current bit is set
        // The mask is 1 shifted left by 'i' positions
        if ((byte >> i) & 1) { 
            printf("1");
        } else {
            printf("0");
        }
    }
    printf("\r\n");
}

void dump_bits(uint8_t reg, const struct bit_name *tbl)
{
    for (int i = 0; i <= 7; i++) {
        if (tbl[i].name == NULL)
            continue;  // skip unused bits             

        printf("%s = %d\n", tbl[i].name, !!(reg & tbl[i].mask));
    }
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

/*---- FUSB302 functions ----*/

static void fusb_delay_ms(uint32_t ms) {
    for (volatile uint32_t i=0; i<ms*4800; i++);
}

static void fusb_reset(uint32_t i2c) {
    fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(10);
}

static void fusb_pd_reset(uint32_t i2c) {
    fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_PD);
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

static int fusb_measure_cc_pin_src(uint32_t i2c, uint8_t cc_reg) {
    // Read status from switches0 register
    uint8_t reg, sw0_orig, cc_lvl;
    fusb_read_reg(i2c, FUSB302_REG_SWITCHES0, &reg);
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
    fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, reg);
    // Set MDAC to default value
    uint8_t mdac = FUSB302_MEAS_MDAC_MV(PD_SRC_DEF_MV);
    fusb_write_reg(i2c, FUSB302_REG_MEASURE, mdac);
    fusb_delay_ms(250);
    // Read status register
    fusb_read_reg(i2c, FUSB302_REG_STATUS0, &reg);
    // Assume open
    cc_lvl = 0;
    // CC voltage below no connect threshold
    if ((reg & FUSB302_STATUS0_COMP) == 0) {
        fusb_write_reg(i2c, FUSB302_REG_MEASURE, PD_SRC_DEF_RD_MV);
        fusb_delay_ms(250);

        // Read status register
        fusb_read_reg(i2c, FUSB302_REG_STATUS0, &reg);

        cc_lvl = (reg & FUSB302_STATUS0_COMP) ? TYPEC_CC_VOLT_RD : TYPEC_CC_VOLT_RA;
    }
    // Restore original switches0 register
    fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, sw0_orig);
    return cc_lvl;
}

static int fusb_check_cc_lines(int32_t i2c) {
    int ret = 0;
    uint8_t status0;
    fusb_read_reg(i2c, FUSB302_REG_STATUS0, &status0);
    if (status0 & FUSB302_STATUS0_COMP) { // COMP bit indicates something attached
        /*BC_LVL is only defined when Measure block is on which is when
          register bits PWR[2]=1 and either MEAS_CC1=1 or MEAS_CC2=1*/
        uint8_t bc_lvl = status0 & FUSB302_STATUS0_BC_LVL_MASK;
        printf("Device detected on CC lines. BC_LVL: 0x%02X\n", bc_lvl);

        // Determine which CC line has the device
        // Measure CC1 only: MEAS_CC1=1, PU_EN1=1
        fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_PU_EN1);
        fusb_read_reg(i2c, FUSB302_REG_STATUS0, &status0);
        printf("FUSB302 STATUS0 after CC1 measure: 0x%02X\n", status0);
        uint8_t cc1_level = status0 & FUSB302_STATUS0_BC_LVL_MASK;
        printf("CC1 BC_LVL: 0x%02X\n", cc1_level);

        // Measure CC2 only: MEAS_CC2=1, PU_EN2=1
        fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC2 | FUSB302_SW0_PU_EN2);
        fusb_read_reg(i2c, FUSB302_REG_STATUS0, &status0);
        printf("FUSB302 STATUS0 after CC2 measure: 0x%02X\n", status0);
        uint8_t cc2_level = status0 & FUSB302_STATUS0_BC_LVL_MASK;
        printf("CC2 BC_LVL: 0x%02X\n", cc2_level);

        // Configure for detected orientation
        if (cc1_level > 0x00 && cc2_level == 0x00) {
            // Return 1 for CC1
            ret = 1;
            // Device on CC1
            printf("Device detected on CC1. BC_LVL: 0x%02X\n", cc1_level);
            // PU_EN1=1, MEAS_CC1=1
            fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, FUSB302_SW0_PU_EN1 | FUSB302_SW0_MEAS_CC1);
            // TXCC1=1, AUTO_CRC=1
            fusb_write_reg(i2c, FUSB302_REG_SWITCHES1, FUSB302_SW1_TXCC1 | FUSB302_SW1_AUTO_CRC);
        } else if (cc2_level > 0x00 && cc1_level == 0x00) {
            // Return 2 for CC2
            ret = 2;
            // Device on CC2
            printf("Device detected on CC2. BC_LVL: 0x%02X\n", cc2_level);
            // PU_EN2=1, MEAS_CC2=1
            fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, FUSB302_SW0_PU_EN2 | FUSB302_SW0_MEAS_CC2);
            // TXCC2=1, AUTO_CRC=1
            fusb_write_reg(i2c, FUSB302_REG_SWITCHES1, FUSB302_SW1_TXCC2 | FUSB302_SW1_AUTO_CRC);
        }
    } else {
        printf("No device detected on CC lines.\n");
        // Return 0 if no device detected
        ret = 0;
    }
    return ret;
}

static void fusb_setup_sniffer(int32_t i2c) {
    uint8_t res, clear_mask;
    
    printf("Initializing FUSB302 for PD Sniffing...\n");
    
    // Reset the FUSB302
    fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(10);

    // Power on
    fusb_write_reg(i2c, FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);

    // Unmask all interrupts 
    fusb_write_reg(i2c, FUSB302_REG_MASK, 0x00);
    fusb_write_reg(i2c, FUSB302_REG_MASKA, 0x00);
    fusb_write_reg(i2c, FUSB302_REG_MASKB, 0x00);

    // Configure listening mode
    // Flush RX
    fusb_read_reg(i2c, FUSB302_REG_CONTROL1, &res);
    res |= FUSB302_CTL1_RX_FLUSH;
    fusb_write_reg(i2c, FUSB302_REG_CONTROL1, res);

    // Disable pull-downs
    fusb_read_reg(i2c, FUSB302_REG_SWITCHES0, &res);
    clear_mask = ~(FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2) & 0xFF;
    res &= clear_mask;
    fusb_write_reg(i2c, FUSB302_REG_SWITCHES0, res);
    fusb_delay_ms(10);

    // Enable SOP' and SOP''
    fusb_read_reg(i2c, FUSB302_REG_CONTROL1, &res);
    res |= (FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB);
    fusb_write_reg(i2c, FUSB302_REG_CONTROL1, res);

    // Flush TX
    fusb_read_reg(i2c, FUSB302_REG_CONTROL0, &res);
    res |= FUSB302_CTL0_TX_FLUSH;
    fusb_write_reg(i2c, FUSB302_REG_CONTROL0, res);

    // Flush RX again?
    fusb_read_reg(i2c, FUSB302_REG_CONTROL1, &res);
    res |= FUSB302_CTL1_RX_FLUSH;
    fusb_write_reg(i2c, FUSB302_REG_CONTROL1, res);

    // Reset PD
    fusb_write_reg(i2c, FUSB302_REG_RESET, FUSB302_RESET_PD);
    fusb_delay_ms(10);

    printf("FUSB302 configured for PD Sniffing.\n");
}

/* CLI parser */
static void handle_command(char *line) {
    if (line[0] == 'r') {
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val;
        fusb_read_reg(I2C1, reg, &val);
        printf("read[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 'w') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t val = (uint8_t)strtol(p, NULL, 0);
        fusb_write_reg(I2C1, reg, val);
        printf("write[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 'p') {
        printf("Scanning I2C...\r\n");
        for (uint8_t addr=1; addr<0x7F; addr++) {
            if (i2c_probe_addr(I2C1, addr)) {
                printf("Probed 0x%02X\r\n", addr);
            }
        }
    } else if (line[0] == 'b') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("bulk read usage: b <reg>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        size_t nbytes = 80;
        uint8_t buf[80]; 
        fusb_read_reg_nbytes(I2C1, reg, buf, nbytes);
        printf("read[0x%02X] = ", reg);
        for (int i=0; i<80; i++) {
            printf("0x%02X ", buf[i]);
        }
    } else if (line[0] == 'n') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("bulk write usage: n <reg> <val1> <val2> ...\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        uint8_t buf[40];
        size_t nbytes = 0;
        while ((p = strtok(NULL, " ")) != NULL && nbytes < sizeof(buf)) {
            buf[nbytes++] = (uint8_t)strtol(p, NULL, 0);
        }
        fusb_write_reg_nbytes(I2C1, reg, buf, nbytes);
        printf("bulk wrote %u bytes to reg 0x%02X\r\n", (unsigned)nbytes, reg);
    } else if (line[0] == 't') {
        // Print bits set in register
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val;
        fusb_read_reg(I2C1, reg, &val);
        print_byte_as_bits(val, reg);
    } else if (line[0] == 'c') {
        // Call a function by name
        char *p = strtok(&line[1], " ");
        if (!p) { printf("usage: c <function_name>\r\n"); return; }
        if (strcmp(p, "fusb_measure_cc_pin_src") == 0) {
            int cc1_lvl = fusb_measure_cc_pin_src(I2C1, FUSB302_SW0_MEAS_CC1);
            int cc2_lvl = fusb_measure_cc_pin_src(I2C1, FUSB302_SW0_MEAS_CC2);
            printf("CC1 level: %d, CC2 level: %d\r\n", cc1_lvl, cc2_lvl);
        } else if (strcmp(p, "fusb_check_cc_lines") == 0) {
            int ret = fusb_check_cc_lines(I2C1);
            if (ret == 1) {
                printf("Device detected on CC1.\r\n");
            } else if (ret == 2) {
                printf("Device detected on CC2.\r\n");
            } else {
                printf("No device detected on CC lines.\r\n");
            }
        } else if (strcmp(p, "fusb_get_chip_id") == 0) {
            uint8_t id = fusb_get_chip_id(I2C1);
            printf("FUSB302 Chip ID (Reg: 0x01): 0x%02X\r\n", id);
        } else if (strcmp(p, "fusb_reset") == 0) {
            fusb_reset(I2C1);
            printf("FUSB302 Reset (Reg: 0x0C) performed\r\n");
        } else if (strcmp(p, "fusb_power_all") == 0) {
            fusb_power_all(I2C1);
            printf("FUSB302 Power (Reg: 0x0B) all on\r\n");
        } else if (strcmp(p, "fusb_pd_reset") == 0) {
            fusb_pd_reset(I2C1);
            printf("FUSB302 PD Reset (Reg: 0x0C) performed\r\n");
        } else if (strcmp(p, "fusb_setup_sniffer") == 0) {
            fusb_setup_sniffer(I2C1);
            printf("FUSB302 Sniffer mode setup done\r\n");
        } else {
            printf("Unknown function: %s\r\n", p);
        }
    } else {
        printf("Commands:\r\n  Read from register:\t\tr <reg>\r\n  Write to register:\t\tw <reg> <val>\r\n  Probe I2C addresses:\t\tp (probe)\r\n  Bulk read:\t\t\tb <reg>\r\n  Bulk write to register:\tn <reg> <val1> <val2> ...\r\n  Read bits in register:\tt <reg> \r\n  Call function:\t\tc <name> \r\n");
    }
}

int main(void) {
    clock_setup();
    usart_setup();
    i2c_setup();

    printf("---- PD Debugger ----\r\n");

    char line[32]; int pos=0;
    while (1) {
        char c = usart_getc();
        if (c=='\r' || c=='\n') {
            line[pos]=0;
            printf("\r\n");
            handle_command(line);
            pos=0;
            printf("> ");
        } else if (pos < (int)sizeof(line)-1) {
            usart_send_blocking(USART2, c); /* echo */
            line[pos++]=c;
        }
    }
}
