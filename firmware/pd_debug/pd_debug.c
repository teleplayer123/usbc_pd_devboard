#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <stdio.h>
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

static void print_byte_as_bits(uint8_t byte) {
    for (int i = 7; i >= 0; i--) { // Loop from most significant bit (7) to least significant (0)
        // Use a bitwise AND with a mask to check if the current bit is set
        // The mask is 1 shifted left by 'i' positions
        if ((byte >> i) & 1) { 
            printf("1");
        } else {
            printf("0");
        }
    }
    printf("\n");
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

void fusb_setup_sniffer(uint32_t i2c) {
    fusb_write_reg(i2c, FUSB302_REG_SWITCHES0,
        FUSB302_SW0_PU_EN2 |
        FUSB302_SW0_MEAS_CC1);
    fusb_write_reg(i2c, FUSB302_REG_SWITCHES1,
        FUSB302_SW1_POWERROLE |   // source
        FUSB302_SW1_SPECREV1 |    // spec rev 2.0
        FUSB302_SW1_DATAROLE |    // DFP
        FUSB302_SW1_AUTO_CRC);    // auto CRC

    fusb_write_reg(i2c, FUSB302_REG_CONTROL3,
        0x00);                    // sniffer mode

    fusb_write_reg(i2c, FUSB302_REG_MASKA,
        0x00);                    // unmask all interrupts

    fusb_write_reg(i2c, FUSB302_REG_MASKB,
        0x00);                    // unmask all interrupts
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
        printf("\r\n");
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
        print_byte_as_bits(val);
    } else if (line[0] == 's') {
        // Setup FUSB302 as sniffer
        fusb_setup_sniffer(I2C1);
        printf("FUSB302 configured as PD sniffer\r\n");
    } else {
        printf("Commands:\r\n  r <reg>\r\n  w <reg> <val>\r\n  p (probe)\r\n  b <reg>\r\n  n <reg> <val1> <val2> ...\r\n  t <reg> (check register set bits)\r\n  s (setup sniffer)\r\n");
    }
}

int main(void) {
    clock_setup();
    usart_setup();
    i2c_setup();
    printf("\r\nFUSB302 I2C test\r\n> ");

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
