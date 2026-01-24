#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fusb302.h"
#include "usb_cdcacm.h"

volatile uint32_t system_millis;

void sys_tick_handler(void)
{
    system_millis++;
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);
    // Enable the clock for SYSCFG to configure EXTI.
    // rcc_periph_clock_enable(RCC_SYSCFG_COMP);

}

static void usb_setup(void)
{
    /* Clock Setup for F072 Crystal-less USB */
    rcc_periph_clock_enable(RCC_CRS);
    crs_autotrim_usb_enable(); // Enable auto-trimming from USB SOF
    rcc_set_usbclk_source(RCC_HSI48);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF0, GPIO11 | GPIO12);
}

static void i2c_setup(void)
{
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    // gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO8);

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

// static void exti_setup(void)
// {
//     // Map PB8 to EXTI8
//     exti_select_source(EXTI8, GPIOB);
//     // Set EXTI8 to trigger on a falling edge (INT_N is active-low)
//     exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
//     // Enable EXTI8 interrupt line
//     exti_enable_request(EXTI8);
//     nvic_enable_irq(NVIC_EXTI4_15_IRQ); 
// }

static void fusb_read(uint8_t reg, uint8_t *val) {
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, val, 1);
}

static void fusb_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    i2c_transfer7(I2C1, FUSB302_ADDR, buf, 2, NULL, 0);
}

/* --- USB CDC interface --- */
static usbd_device *usbdev;

int _write(int fd, char *ptr, int len)
{
    (void)fd;
    if (!usbdev)
        return len;

    while (len > 0) {
        int chunk = len > 64 ? 64 : len;
        usbd_ep_write_packet(usbdev, 0x82, ptr, chunk);
        ptr += chunk;
        len -= chunk;
    }
    return 0;
}

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
    clock_setup();
    systick_setup();
    i2c_setup();
    usb_setup();
    usbdev = usb_cdcacm_init(cdcacm_rx_cb);

	for (int i = 0; i < 0x800000; i++)
		__asm__("nop");

    while (1) {
        usb_printf("%d\r\n", system_millis);
        usbd_poll(usbdev);
    }
}
