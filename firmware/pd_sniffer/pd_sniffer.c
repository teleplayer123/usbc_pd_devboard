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
volatile bool fusb_event_pending;

typedef struct {
    uint16_t header;
    uint32_t obj[7];
} pd_msg_t;

pd_msg_t last_caps;
int last_pdo_count = 0;

/* ---------------- State ---------------- */
static uint8_t last_cc_state;
static bool    vbus_present;
static uint32_t vbus_rise_ms;
static bool    first_pd_seen;

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
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);
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
 * Low-level helpers
 * ------------------------------------------------------------ */

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

static bool uart_rx_ready(void)
{
    return usart_get_flag(USART2, USART_FLAG_RXNE);
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

static void fusb_setup_sniffer() {    
    usart_printf("Initializing FUSB302 for PD Sniffing...\n");
    
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

    usart_printf("FUSB302 configured for PD Sniffing.\n");
}

/* ============================================================
 * EXTI — PB8 / INT_N  
 * ============================================================ */

void exti4_15_isr(void)
{
    if (exti_get_flag_status(EXTI8)) {
        fusb_event_pending = true;
        exti_reset_request(EXTI8);
    }
}

/* ============================================================
 * CC MEASUREMENT 
 * ============================================================ */

static uint8_t measure_cc(bool cc1)
{
    uint8_t sw0 = fusb_read(FUSB302_REG_SWITCHES0);

    /* Save current Rd state */
    uint8_t saved_pdwn =
        sw0 & (FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);

    /* Disable Rd while measuring */
    sw0 &= ~(FUSB302_SW0_PDWN1 | FUSB302_SW0_PDWN2);

    /* Select CC to measure */
    sw0 &= ~(FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);
    sw0 |= cc1 ? FUSB302_SW0_MEAS_CC1 : FUSB302_SW0_MEAS_CC2;

    fusb_write(FUSB302_REG_SWITCHES0, sw0);

    /* Comparator settle time */
    for (volatile int i = 0; i < 500; i++) __asm__("nop");

    uint8_t st = fusb_read(FUSB302_REG_STATUS0);
    uint8_t bc = st & 0x03;  /* BC_LVL */

    /* Restore Rd */
    fusb_write(FUSB302_REG_SWITCHES0,
               (sw0 & ~(FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2)) |
               saved_pdwn);

    return bc;
}


static void log_cc_state(void)
{
    uint8_t bc1 = measure_cc(true);
    uint8_t bc2 = measure_cc(false);

    uint8_t state = (bc1 ? 1 : 0) | (bc2 ? 2 : 0);
    if (state == last_cc_state)
        return;

    last_cc_state = state;

    if (bc1 && !bc2) {
        usart_printf("[CC] Attached on CC1 (Rp=%u)", bc1);
    } else if (!bc1 && bc2) {
        usart_printf("[CC] Attached on CC2 (Rp=%u)", bc2);
    } else if (!bc1 && !bc2) {
        usart_printf("[CC] Detached");
    } else {
        usart_printf("[CC] Invalid (Rp on both CCs)");
    }
}

/* ============================================================
 * VBUS LOGGING
 * ============================================================ */

static void poll_vbus(void)
{
    uint8_t st = fusb_read(FUSB302_REG_STATUS0);
    bool now = st & FUSB302_STATUS0_VBUSOK;

    if (now && !vbus_present) {
        vbus_present = true;
        vbus_rise_ms = system_millis;
        first_pd_seen = false;
        usart_printf("[VBUS] PRESENT at %lu ms", vbus_rise_ms);
    }

    if (!now && vbus_present) {
        vbus_present = false;
        usart_printf("[VBUS] REMOVED at %lu ms", system_millis);
    }
}

/* ============================================================
 * PD RX (token‑aware, SOP only)
 * ============================================================ */

static void log_first_pd_timing(void)
{
    if (!first_pd_seen && vbus_present) {
        uint32_t delta = system_millis - vbus_rise_ms;
        usart_printf("[PD] First SOP packet after %lu ms from VBUS rise", delta);
        first_pd_seen = true;
    }
}

bool read_pd_message(pd_msg_t *pd)
{
    uint8_t tok;

    fusb_read_fifo(&tok, 1);
    if ((tok & 0xE0) != FUSB302_RX_TKN_SOP)
        return false;

    fusb_read_fifo(&tok, 1);
    if ((tok & 0xE0) != FUSB302_RX_TKN_PACKSYM)
        return false;

    uint8_t hdr[2];
    fusb_read_fifo(hdr, 2);
    pd->header = hdr[0] | (hdr[1] << 8);

    int nobj = PD_HEADER_NUM_DATA_OBJECTS(pd->header);
    if (nobj > 7) nobj = 7;

    if (nobj) {
        fusb_read_fifo(&tok, 1);
        fusb_read_fifo((uint8_t *)pd->obj, nobj * 4);
    }

    do {
        fusb_read_fifo(&tok, 1);
    } while ((tok & 0xD0) != FUSB302_RX_TKN_EOP);

    log_first_pd_timing();
    return true;
}

static const char *pd_msg_name(uint8_t type, bool data)
{
    if (!data) {
        switch (type) {
        case 0:  return "GoodCRC";
        case 1:  return "GotoMin";
        case 2:  return "Accept";
        case 3:  return "Reject";
        case 6:  return "PS_RDY";
        case 7:  return "Get_Source_Cap";
        case 8:  return "Get_Sink_Cap";
        case 13: return "Soft_Reset";
        default: return "Ctrl_Unknown";
        }
    } else {
        switch (type) {
        case 1:  return "Source_Capabilities";
        case 2:  return "Request";
        case 4:  return "Sink_Capabilities";
        case 15: return "Vendor_Defined";
        default: return "Data_Unknown";
        }
    }
}

static void pd_log_message(pd_msg_t *p)
{
    uint8_t type  = PD_HEADER_MESSAGE_TYPE(p->header);
    uint8_t nobj  = PD_HEADER_NUM_DATA_OBJECTS(p->header);
    bool data     = (nobj > 0);

    uint8_t msgid = (p->header >> 9) & 0x7;
    uint8_t prole = (p->header >> 8) & 0x1;
    uint8_t drole = (p->header >> 5) & 0x1;
    uint8_t rev   = (p->header >> 6) & 0x3;

    usart_printf(
        "[%8lu ms] PD RX | %s | ID=%d | %s | %s | Rev=%d | Obj=%d | HDR=0x%04X\r\n",
        system_millis,
        pd_msg_name(type, data),
        msgid,
        prole ? "SRC" : "SNK",
        drole ? "DFP" : "UFP",
        rev,
        nobj,
        p->header
    );

    for (int i = 0; i < nobj; i++) {
        usart_printf("    OBJ%d: 0x%08lX\r\n", i + 1, p->obj[i]);

        if (type == 1) {
            int mv = ((p->obj[i] >> 10) & 0x3FF) * 50;
            int ma = (p->obj[i] & 0x3FF) * 10;
            usart_printf("        -> %d mV @ %d mA\r\n", mv, ma);
        }

        if (type == 2) {
            int pdo = (p->obj[i] >> 28) & 0x7;
            int ma  = ((p->obj[i] >> 10) & 0x3FF) * 10;
            int mv  = (p->obj[i] & 0x3FF) * 50;
            usart_printf("        -> Request PDO%d %d mV %d mA\r\n",
                          pdo, mv, ma);
        }
    }
}

/* ============================================================
 * MAIN POLL ENTRY
 * ============================================================ */

static void fusb_poll(void)
{
    log_cc_state();
    poll_vbus();

    if (!fusb_rx_empty()) {
        pd_msg_t msg;
        if (read_pd_message(&msg)) {
            pd_log_message(&msg);
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

    fusb_setup_sniffer();

    while (1) {
        // debug in main loop
        usart_printf("Systick: %02X\r\n", system_millis);
        fusb_poll();
        fusb_delay_ms(500);
    }
}
