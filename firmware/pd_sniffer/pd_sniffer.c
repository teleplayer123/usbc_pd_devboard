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
#include "fusb302.h"

// Buffer to store the raw packet data
uint8_t rx_buffer[MAX_PD_PACKET_SIZE];
// Status flag for exti handler
volatile bool fusb_event_pending = false;

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

void exti4_15_isr(void) {
    exti_reset_request(EXTI8);  // clear interrupt flag
    fusb_event_pending = true;  // signal main loop to handle PD
}

static void check_rx_buffer(void) {
    hexdump(rx_buffer, MAX_PD_PACKET_SIZE);
}

int main(void) {
    clock_setup();
    systick_setup();
    usart_setup();
    i2c_setup();
    exti_setup();
    fusb_init_sink(I2C1); // Initially setup as a sink                

    while (1) {
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
    return 0;
}
