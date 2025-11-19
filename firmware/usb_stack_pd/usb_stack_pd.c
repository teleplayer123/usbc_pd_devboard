#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usb_cdcacm.h"

#define FUSB302_ADDR 0x22

static void i2c_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);

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

static void fusb_read(uint8_t reg, uint8_t *val) {
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, val, 1);
}

static void fusb_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_transfer7(I2C1, FUSB302_ADDR, buf, 2, NULL, 0);
}

/* --- USB CDC interface --- */
static usbd_device *usbdev;

static void cdcacm_rx_cb(uint8_t *buf, int len) {
    /* Simple command parser: r <reg> | w <reg> <val> */
    if (len < 2) return;

    if (buf[0] == 'r') {
        uint8_t reg = strtol((char*)&buf[1], NULL, 0);
        uint8_t val;
        fusb_read(reg, &val);
        char out[32];
        int n = snprintf(out, sizeof(out), "R[0x%02X]=0x%02X\r\n", reg, val);
        usb_cdcacm_write((uint8_t*)out, n);
    } else if (buf[0] == 'w') {
        char *p = strtok((char*)&buf[1], " ");
        if (!p) return;
        uint8_t reg = strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) return;
        uint8_t val = strtol(p, NULL, 0);
        fusb_write(reg, val);
        char out[32];
        int n = snprintf(out, sizeof(out), "W[0x%02X]=0x%02X\r\n", reg, val);
        usb_cdcacm_write((uint8_t*)out, n);
    }
}

int main(void) {
    rcc_clock_setup_in_hsi48_out_48mhz();
    rcc_periph_clock_enable(RCC_USB);

    i2c_setup();
    usbdev = usb_cdcacm_init(cdcacm_rx_cb);

    while (1) {
        usbd_poll(usbdev);
    }
}
