#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define FUSB302_ADDR  0x22   // 7-bit address

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

/* simple blocking getchar/putchar */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}
static char usart_getc(void) { 
    return usart_recv_blocking(USART2); 
}

/* I2C helpers */
/*
void i2c_transfer7(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn);
    i2c: The base address of the I2C peripheral
    addr: The 7-bit I2C slave address
    w: A pointer to the data buffer to be written to the slave
    wn: The number of bytes to write
    r: A pointer to the buffer where the data read from the slave will be stored
    rn: The number of bytes to read from the slave
*/

static int fusb_read_reg(uint8_t reg, uint8_t *val) {
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, val, 1);
    return 0;
}
static int fusb_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_transfer7(I2C1, FUSB302_ADDR, buf, 2, NULL, 0);
    return 0;
}

static int fusb_read_reg_nbytes(uint8_t reg, uint8_t *buf, size_t nbytes) {
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, buf, nbytes);
    return 0;
}

static void fusb_write_reg_nbytes(uint8_t reg, const uint8_t *buf, size_t nbytes) {
    uint8_t wbuf_size = 1 + nbytes;
    uint8_t *wbuf = malloc(wbuf_size);
    wbuf[0] = reg;
    memcpy(&wbuf[1], buf, nbytes);
    i2c_transfer7(I2C1, FUSB302_ADDR, wbuf, 1 + nbytes, NULL, 0);
}

static bool fusb_check_i2c_addr(uint8_t addr) {
    i2c_transfer7(I2C1, addr, NULL, 0, NULL, 0);
    bool ack = !(I2C_ISR(I2C1) & I2C_ISR_NACKF);
    return ack;
}

/* low level i2c scan */
static bool i2c_probe_addr(uint8_t addr) {

    /* clear flags */
    I2C_ICR(I2C1) = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    /* send address */
    I2C_CR2(I2C1) =
        (addr << 1) |      // address in bits 7:1
        (0 << 16)  |       // number of bytes
        I2C_CR2_START;     // generate START

    /* wait for either ACK or NACK */
    while (1) {
        uint32_t isr = I2C_ISR(I2C1);

        if (isr & I2C_ISR_NACKF) {
            I2C_ICR(I2C1) = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
            return false;  // NACK → no device
        }

        if (isr & I2C_ISR_STOPF) {
            I2C_ICR(I2C1) = I2C_ICR_STOPCF;
            return true;   // STOP with no NACK → device responded
        }
    }
}


/* CLI parser */
static void handle_command(char *line) {
    if (line[0] == 'r') {
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val;
        fusb_read_reg(reg, &val);
        printf("read[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 'w') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t val = (uint8_t)strtol(p, NULL, 0);
        fusb_write_reg(reg, val);
        printf("write[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 's') {
        printf("Scanning I2C...\r\n");
        for (uint8_t addr=1; addr<0x7F; addr++) {
            if (i2c_probe_addr(addr)) {
                printf("Probed 0x%02X\r\n", addr);
            }
        }
    } else if (line[0] == 'b') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("bulk read usage: b <reg>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        size_t nbytes = 80;
        uint8_t buf[80]; 
        fusb_read_reg_nbytes(reg, buf, nbytes);
        printf("read[0x%02X] = ", reg);
        for (int i=0; i<80; i++) {
            printf("0x%02X ", buf[i]);
        }
        printf("\r\n");
    } else {
        printf("Commands:\r\n  r <reg>\r\n  w <reg> <val>\r\n  s (scan)\r\n  b <reg>\r\n");
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
