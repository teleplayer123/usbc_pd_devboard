#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "fusb302.h"

/* -------------------------------------------------------------------------- */
/* Global timebase                                                             */
/* -------------------------------------------------------------------------- */

volatile uint32_t system_millis;

void sys_tick_handler(void)
{
    system_millis++;
}

/* -------------------------------------------------------------------------- */
/* USB CDC                                                                    */
/* -------------------------------------------------------------------------- */
static usbd_device *usbdev;
#define USB_RX_BUF_SZ 256
static uint8_t usb_rx_buf[USB_RX_BUF_SZ];
static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;

static void usb_write(const char *s, int len)
{
    while (len > 0) {
        int n = usbd_ep_write_packet(usbdev, 0x82, s, len);
        if (n > 0) {
            s   += n;
            len -= n;
        }
    }
}

int _write(int fd, const char *buf, int len)
{
    (void)fd;

    /* Route stdout/stderr to USB CDC */
    usb_write(buf, len);

    return len;
}

static void usb_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0)
        usb_write(buf, n);
}

static int usb_getc_nonblocking(void)
{
    if (usb_rx_head == usb_rx_tail)
        return -1;

    int c = usb_rx_buf[usb_rx_tail++];
    usb_rx_tail %= USB_RX_BUF_SZ;
    return c;
}

static void cdc_rx_cb(usbd_device *dev, uint8_t ep)
{
    (void)ep;
    char buf[64];
    int len = usbd_ep_read_packet(dev, 0x01, buf, sizeof(buf));

    for (int i = 0; i < len; i++) {
        usb_rx_buf[usb_rx_head++] = buf[i];
        usb_rx_head %= USB_RX_BUF_SZ;
    }
}

static uint8_t usbd_control_buffer[128];

extern const struct usb_device_descriptor dev_descr;
extern const struct usb_config_descriptor config_descr;
extern const char *usb_strings[];
extern void cdcacm_set_config(usbd_device *dev);

static void usb_setup(void)
{
    rcc_periph_clock_enable(RCC_USB);

    usbdev = usbd_init(
        &st_usbfs_v2_usb_driver,
        &dev_descr,
        &config_descr,
        usb_strings,
        3,
        usbd_control_buffer,
        sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbdev, cdcacm_set_config);
}

static void usb_poll(void)
{
    usbd_poll(usbdev);
}

/* -------------------------------------------------------------------------- */
/* FUSB302 interrupt glue                                                      */
/* -------------------------------------------------------------------------- */

static void clock_setup(void) {
    rcc_clock_setup_in_hsi_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);
}

volatile bool fusb_event_pending;

void exti4_15_isr(void)
{
    if (exti_get_flag_status(EXTI8)) {
        exti_reset_request(EXTI8);
        fusb_event_pending = true;
    }
}

static void exti_setup(void)
{
    /* Enable SYSCFG/COMP clock */
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    /* PB8 as input, pull-up */
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO8);

    /* Map EXTI8 to PB8
     * EXTI8 is in EXTICR3 (EXTI lines 8..11)
     * Port B = 0x1
     */
    SYSCFG_EXTICR3 = (SYSCFG_EXTICR3 & ~(0xF << 0)) |  (0x1 << 0); /* clear EXTI8 bits; PB = 1 */
                              
    /* Configure EXTI8 */
    exti_select_source(EXTI8, GPIOB);
    exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI8);

    nvic_enable_irq(NVIC_EXTI4_15_IRQ);
}

static void i2c_setup(void) {
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);

    // Hardware reset via RCC 
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);

    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

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

static bool fusb_rx_empty(void)
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

static void fusb_init() {    
    // Reset the FUSB302
    fusb_write(FUSB302_REG_RESET, FUSB302_RESET_SW);
    fusb_delay_ms(2);

    // Power on
    fusb_write(FUSB302_REG_POWER, FUSB302_POWER_ALL_ON);
    fusb_delay_ms(2);

    // Enable SOP 
    fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2);

    // Accept SOP packets
    fusb_write(FUSB302_REG_CONTROL2, FUSB302_CTL2_WAKE_EN | FUSB302_CTL2_TOGGLE);

    // Enable CC comparators
    fusb_write(FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);

    // Flush FIFO
    fusb_write(FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_FLUSH);
    fusb_write(FUSB302_REG_CONTROL1, FUSB302_CTL1_RX_FLUSH);

    // Unmask all interrupts 
    fusb_write(FUSB302_REG_MASK, 0x00);
    fusb_write(FUSB302_REG_MASKA, 0x00);
    fusb_write(FUSB302_REG_MASKB, 0x00);

    // Clear pending interrupts
    fusb_read(FUSB302_REG_INTERRUPT);
    fusb_read(FUSB302_REG_INTERRUPTA);
    fusb_read(FUSB302_REG_INTERRUPTB);
    fusb_delay_ms(2);
}

/* -------------------------------------------------------------------------- */
/* PD Packet Handling                                                         */
/* -------------------------------------------------------------------------- */

void fusb_log_pd_packet(const uint8_t *buf, int len)
{
    if (len < 2) {
        usb_printf("  PD: runt packet (%d bytes)\r\n", len);
        return;
    }

    uint16_t hdr = buf[0] | (buf[1] << 8);

    uint8_t msg_type = (hdr >> 12) & 0x0F;
    uint8_t num_obj  = (hdr >> 9)  & 0x07;
    uint8_t pwr_role = (hdr >> 8)  & 0x01;
    uint8_t spec_rev = (hdr >> 6)  & 0x03;
    uint8_t data_role= (hdr >> 5)  & 0x01;
    uint8_t msg_id   =  hdr        & 0x1F;

    usb_printf(
        "  PD hdr: type=%u id=%u objs=%u pr=%u dr=%u rev=%u len=%d\r\n",
        msg_type, msg_id, num_obj, pwr_role, data_role, spec_rev, len
    );

    /* Dump payload */
    for (int i = 2; i < len; i += 4) {
        uint32_t obj = 0;
        for (int j = 0; j < 4 && (i + j) < len; j++)
            obj |= buf[i + j] << (8 * j);

        usb_printf("    OBJ %d: 0x%08lx\r\n",
                   (i - 2) / 4, (unsigned long)obj);
    }
}

void fusb_log_status(void)
{
    uint8_t st0 = fusb_read(FUSB302_REG_STATUS0);
    uint8_t st1 = fusb_read(FUSB302_REG_STATUS1);

    bool vbus = st0 & FUSB302_STATUS0_VBUSOK;
    uint8_t bc = st0 & 0x03;

    const char *rp_str = "unknown";
    switch (bc) {
    case 0: rp_str = "open"; break;
    case 1: rp_str = "Rp-default"; break;
    case 2: rp_str = "Rp-1.5A"; break;
    case 3: rp_str = "Rp-3.0A"; break;
    }

    usb_printf(
        "[%lu ms] STATUS: VBUS=%s BC_LVL=%u (%s)\r\n",
        system_millis,
        vbus ? "ON" : "OFF",
        bc, rp_str
    );
}

static void fusb_process_rx_token(uint8_t token)
{
    static uint8_t pd_buf[64];
    static int pd_len;
    static bool in_packet;

    if (token == FUSB302_RX_TKN_SOP) {
        in_packet = true;
        pd_len = 0;
        usb_printf("[%lu] RX SOP\r\n", system_millis);
        return;
    }

    if (!in_packet)
        return;

    if (token & FUSB302_RX_TKN_PACKSYM) {
        /* PACKSYM token: lower 5 bits = number of data bytes */
        int count = token & 0x1F;
        for (int i = 0; i < count; i++) {
            pd_buf[pd_len++] = fusb_read(FUSB302_REG_FIFOS);
        }
        return;
    }

    if (token == FUSB302_RX_TKN_JAMCRC) {
        usb_printf("  CRC OK, %d bytes\r\n", pd_len);
        fusb_log_pd_packet(pd_buf, pd_len);
        in_packet = false;
        return;
    }

    if (token == FUSB302_RX_TKN_EOP) {
        in_packet = false;
        return;
    }

    /* Unknown token: ignore but terminate packet */
    in_packet = false;
}

static void fusb_drain_rx_fifo(void)
{
    while (1) {
        uint8_t token = fusb_read(FUSB302_REG_FIFOS);

        /* FIFO empty condition */
        if (token == 0)
            break;

        fusb_process_rx_token(token);
    }
}

void fusb_handle_irq(void)
{
    /* Reading interrupt registers ACKs INT_N */
    uint8_t int0 = fusb_read(FUSB302_REG_INTERRUPT);
    uint8_t int1 = fusb_read(FUSB302_REG_INTERRUPTA);
    uint8_t int2 = fusb_read(FUSB302_REG_INTERRUPTB);

    (void)int0;
    (void)int1;
    (void)int2;

    /* Always attempt to drain RX FIFO */
    fusb_drain_rx_fifo();

    /* Optionally log CC/VBUS changes */
    fusb_log_status();
}

/* -------------------------------------------------------------------------- */
/* CLI                                                                         */
/* -------------------------------------------------------------------------- */

static void handle_command(char *line)
{
    if (!strcmp(line, "help")) {
        usb_printf("Commands:\r\n");
        usb_printf("  help      Show this help\r\n");
        usb_printf("  status    Dump CC/VBUS status\r\n");
        usb_printf("\r\n");
    } else if (!strcmp(line, "status")) {
        fusb_log_status();
    } else {
        usb_printf("Unknown command: %s\r\n", line);
    }
}

/* -------------------------------------------------------------------------- */
/* Main                                                                        */
/* -------------------------------------------------------------------------- */

int main(void)
{
    systick_set_reload(48000 - 1);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();

    clock_setup();
    i2c_setup();
    usb_setup();
    exti_setup();
    fusb_init();

    usb_printf("---- USB-C PD Debugger ----\r\n> ");

    char line[64];
    int pos = 0;

    while (1) {
        usb_poll();

        if (fusb_event_pending) {
            fusb_event_pending = false;
            fusb_handle_irq();
        }

        int c = usb_getc_nonblocking();
        if (c < 0)
            continue;

        if (c == '\r' || c == '\n') {
            line[pos] = 0;
            usb_printf("\r\n");
            handle_command(line);
            pos = 0;
            usb_printf("> ");
        } else if (pos < (int)sizeof(line) - 1) {
            usb_write((char *)&c, 1); /* echo */
            line[pos++] = c;
        }
    }
}
