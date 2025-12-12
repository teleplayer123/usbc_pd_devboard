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
#include <stdbool.h> 

// --- User-defined Header ---
#include "fusb302.h"

// --- Global PD Definitions ---
#define MAX_PD_PACKET_SIZE 30 // Header (2) + Data Objects (7*4=28)
#define PD_MAX_PAYLOAD_SIZE 28 

// Buffer to store the raw packet data
uint8_t rx_buffer[MAX_PD_PACKET_SIZE];
// Status flag for exti handler
volatile bool fusb_event_pending = false;

// --- Custom PD Message Struct (for easy parsing and logging) ---
typedef struct {
    uint16_t header;         
    uint8_t  msg_type;       
    uint8_t  data_role;      
    uint8_t  spec_rev;       
    uint8_t  power_role;     
    uint8_t  data_obj_count; 
    uint8_t  msg_id;         
    uint32_t obj[7]; // Data Objects
} pd_msg_t;

// --- Global Status Variables ---
enum {
    PE_SNIFFER_LISTEN, 
    PE_SNIFFER_ERROR 
} pe_state_t;

static int g_pd_state = PE_SNIFFER_LISTEN;
static uint8_t g_msg_id_counter = 0; // Not used for sniffer, but good practice

/*---- libopencm3 Utility Functions (Assumed to exist in context) ----*/

// Placeholder for I2C functions from pd_debug.c context
uint8_t i2c_read_reg(uint8_t reg);
void i2c_write_reg(uint8_t reg, uint8_t val);
void fusb_read_fifo(uint8_t *data, size_t len);
void fusb_write_reg_nbytes(uint8_t reg, const uint8_t *data, size_t len);

// Placeholder for printf (using your USART setup)
int usart_printf(const char *format, ...);
bool uart_rx_ready(void);
char usart_recv(uint32_t usart);


/*---- Policy Engine Function Prototypes (using the sniffer state machine) ----*/
bool pd_process_rx_fifo(pd_msg_t *msg);
void pd_log_message(const pd_msg_t *msg, const char *direction);
void fusb_setup_sniffer(uint32_t i2c_periph);
void fusb_handle_irq(void);


/*---- FUSB302 Low-Level I2C/Utility Implementations (Assuming these are in pd_debug.c or linked) ----*/
// ... (i2c_read_reg, i2c_write_reg, etc. implementations here) ...

// CRC-32 JAMCRC (Required for validating received packets)
uint32_t pd_calculate_crc(const uint8_t *data, size_t len)
{
    const uint32_t PD_CRC_POLY = 0xEDB88320; 
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x00000001) {
                crc = (crc >> 1) ^ PD_CRC_POLY;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc; 
}


/*---- FUSB302 Driver / HAL Functions (must be implemented) ----*/

/**
 * @brief Configures the FUSB302 for Sniffer (Passive Listener) mode.
 * @param i2c_periph The I2C peripheral (e.g., I2C1).
 */
void fusb_setup_sniffer(uint32_t i2c_periph)
{
    // 1. Full Reset
    i2c_write_reg(FUSB302_REG_RESET, FUSB302_RESET_SW_RES | FUSB302_RESET_PD_RES);
    
    // 2. Power up everything (except VCONN)
    i2c_write_reg(FUSB302_REG_POWER, FUSB302_PWR_ALL_MASK & ~FUSB302_PWR_VCONN_MASK); 
    
    // 3. Set Control Registers
    // Resetting PD_RST and auto-CRC for receive
    i2c_write_reg(FUSB302_REG_CONTROL0, 0x00);
    i2c_write_reg(FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOPC); // Enable SOP' packets (Sniffer needs this)
    i2c_write_reg(FUSB302_REG_CONTROL3, 0x00); // Disable auto-retry and auto-hard-reset

    // 4. Set Switches to listen on both CC lines
    // TX on neither, Measure on both CC1 and CC2 (best for sniffer)
    i2c_write_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);

    // 5. Mask Interrupts
    // Unmask only the critical RX and Hard Reset interrupts
    i2c_write_reg(FUSB302_REG_MASKA, FUSB302_MASKA_ALL_MASK & ~(FUSB302_INTA_RX_STAT | FUSB302_INTA_HARDRST)); 
    i2c_write_reg(FUSB302_REG_MASK, FUSB302_MASK_ALL_MASK & ~FUSB302_INT_CRC_CHK); // Unmask CRC_CHK for messages
    i2c_write_reg(FUSB302_REG_MASKC, FUSB302_MASKC_ALL_MASK); // All C masks on
    
    // 6. Flush TX/RX FIFOs
    i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_RX_FLUSH | FUSB302_CTL0_TX_FLUSH);
    
    usart_printf("FUSB302 initialized for Sniffer Mode.\r\n");
}


/**
 * @brief Processes and validates a received PD packet from the FIFO.
 * @param msg Pointer to the structure to fill with parsed message data.
 * @return true if a valid packet was processed, false otherwise.
 */
bool pd_process_rx_fifo(pd_msg_t *msg)
{
    // 1. Read the first two bytes (Header) from the FIFO
    uint8_t header_bytes[2];
    fusb_read_fifo(header_bytes, 2); 
    msg->header = header_bytes[0] | (header_bytes[1] << 8);

    // 2. Extract length information
    msg->data_obj_count = PD_HEADER_NUM_DATA_OBJECTS(msg->header);
    int data_len = (msg->data_obj_count * 4) + 4; // Data(DOC*4) + CRC(4)
    if (data_len > PD_MAX_PAYLOAD_SIZE + 4) { // 28 bytes data + 4 bytes CRC
        i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_RX_FLUSH);
        return false;
    }

    // 3. Read remaining bytes (Data Objects and CRC)
    uint8_t rx_data_payload[PD_MAX_PAYLOAD_SIZE + 4];
    fusb_read_fifo(rx_data_payload, data_len);
    
    // 4. Prepare data for CRC check (Header + Data)
    uint8_t crc_check_buf[2 + data_len - 4]; // Header(2) + Data(N*4)
    memcpy(crc_check_buf, header_bytes, 2);
    memcpy(crc_check_buf + 2, rx_data_payload, data_len - 4); 

    // 5. Check CRC
    uint32_t received_crc = rx_data_payload[data_len-4] | (rx_data_payload[data_len-3] << 8) |
                            (rx_data_payload[data_len-2] << 16) | (rx_data_payload[data_len-1] << 24);
    
    uint32_t calculated_crc = pd_calculate_crc(crc_check_buf, sizeof(crc_check_buf));

    // 6. Flush RX FIFO immediately (required after any successful read)
    i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_RX_FLUSH);

    if (received_crc != calculated_crc) return false; 

    // 7. Final Message Parsing
    msg->msg_type = PD_HEADER_MESSAGE_TYPE(msg->header);
    msg->data_role = PD_HEADER_DATA_ROLE(msg->header);
    msg->spec_rev = PD_HEADER_SPEC_REV(msg->header);
    msg->power_role = PD_HEADER_POWER_ROLE(msg->header);
    msg->msg_id = PD_HEADER_MESSAGE_ID(msg->header);
    
    // 8. Copy Data Objects
    for (int i = 0; i < msg->data_obj_count; i++) {
        msg->obj[i] = rx_data_payload[i*4] | (rx_data_payload[i*4+1] << 8) |
                      (rx_data_payload[i*4+2] << 16) | (rx_data_payload[i*4+3] << 24);
    }

    // *** CRITICAL STEP FOR LOGGING ***
    pd_log_message(msg, "RX");
    
    return true; 
}


/**
 * @brief Logs the contents of a PD message to the serial port.
 */
void pd_log_message(const pd_msg_t *msg, const char *direction)
{
    const char *msg_type_str[] = {
        "Control", "Source_Caps", "Request", "BIST", "Sink_Caps", 
        "Vendor_Def", "Control_Reserved"
    };
    
    // PD Control Message Types 
    const char *ctrl_msg_str[] = {
        "Reserved", "GoodCRC", "GotoMin", "Accept", "Reject", "Wait", 
        "PS_RDY", "Soft_Reset", "Not_Supported", "Get_Source_Cap", 
        "Get_Sink_Cap", "DR_Swap", "PR_Swap", "VCONN_Swap", "Send_DR_Swap",
        "Send_PR_Swap"
    };

    usart_printf("[%s] ID:%u Rev:%u %s", direction, msg->msg_id, msg->spec_rev, msg->data_obj_count > 0 ? "Data" : "Control");
    
    if (msg->data_obj_count == 0) {
        // Control Message
        if (msg->msg_type < 16) {
            usart_printf("/%s", ctrl_msg_str[msg->msg_type]);
        }
    } else {
        // Data Message
        if (msg->msg_type < 6) {
             usart_printf("/%s", msg_type_str[msg->msg_type]);
        }
    }
    
    usart_printf(" (Role:%s%s, Type:%u, %u DOs)\r\n", 
        msg->power_role ? "SRC" : "SNK",
        msg->data_role ? "/DFP" : "/UFP",
        msg->msg_type, msg->data_obj_count);
    
    // Log data objects
    for (int i = 0; i < msg->data_obj_count; i++) {
        usart_printf("  PDO%d: 0x%08X\r\n", i, msg->obj[i]);
    }
}


/**
 * @brief Simple function to read a received packet and log its contents.
 */
static void read_pd_message(void)
{
    pd_msg_t rx_message;

    if (pd_process_rx_fifo(&rx_message)) {
        // Packet was valid, logged in pd_process_rx_fifo. 
        // No further action needed for sniffer state.
    } else {
        usart_printf("RX Error: CRC/Length Failure.\r\n");
    }
}


/**
 * @brief The main IRQ handler for the FUSB302.
 */
void fusb_handle_irq(void)
{
    // Reading these clears the interrupt flags
    uint8_t interrupt_a = i2c_read_reg(FUSB302_REG_INTERRUPTA);
    uint8_t interrupt_c = i2c_read_reg(FUSB302_REG_INTERRUPT); 
    
    // A. Handle Hard Reset Received (Resets the sniffer)
    if (interrupt_a & FUSB302_INTA_HARDRST) {
        usart_printf("!!! Hard Reset Received. Re-initializing sniffer...\r\n");
        // Re-initialize FUSB302 for sniffer mode
        fusb_setup_sniffer(I2C1); 
        g_pd_state = PE_SNIFFER_LISTEN;
    }
    
    // B. Handle RX Status/Message Received (CRC_CHK)
    if (interrupt_c & FUSB302_INT_CRC_CHK) {
        read_pd_message();
    }
    
    // C. Handle TX Failure/Retry Fail (Should not happen in passive mode)
    if (interrupt_a & FUSB302_INTA_RETRYFAIL) {
        usart_printf("!!! TX Failure (unexpected in Sniffer mode). Resetting...\r\n");
        fusb_setup_sniffer(I2C1); 
        g_pd_state = PE_SNIFFER_LISTEN;
    }

    // D. Connection/Disconnect event (COMP_CHNG)
    if (interrupt_c & FUSB302_INT_COMP_CHNG) {
        // In sniffer mode, this indicates a cable connection/disconnection event.
        usart_printf("--- Connection/CC Status Change ---\r\n");
    }
}


/*---- Existing MCU Setup Functions (from your original pd_debug.c) ----*/

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
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

int usart_printf(const char *format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    for (int i = 0; i < len; i++) {
        usart_send_blocking(USART2, buf[i]);
    }
    return len;
}

static void i2c_setup(void) {
    // I2C1 pins: SCL=PB6, SDA=PB7
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    
    i2c_reset(I2C1);
    i2c_peripheral_disable(I2C1);
    
    // Standard I2C configuration... (simplified)
    i2c_set_speed(I2C1, i2c_speed_sm_100k, 48); // Assuming 48MHz clock
    
    i2c_peripheral_enable(I2C1);
}

static void exti_setup(void) {
    // FUSB302 INT_N is on PB8 (Pin 44 of STM32F072)
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO8);
    
    exti_select_source(EXTI8, GPIOB);
    exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI8);
    
    nvic_enable_irq(NVIC_EXTI4_15_IRQ);
}

void exti4_15_isr(void) {
    if (exti_get_flag_status(EXTI8)) {
        fusb_event_pending = true;
        exti_clear_flag_status(EXTI8);
    }
}

// ... (Other functions like systick_setup, uart_rx_ready, usart_recv, handle_command) ...

// === COMMAND HANDLER AND MAIN LOOP ===

// Renaming the function to reflect its passive role
static void handle_pd_message(pd_msg_t *p){
    // All logging is now handled by pd_log_message() called inside pd_process_rx_fifo()
    // This function can now be removed or kept as a simple stub if called elsewhere
    return;
}

// *** CRITICAL CHANGE: The main loop now calls the dedicated IRQ handler when an event occurs. ***
int main(void) {
    clock_setup();
    // systick_setup(); // Uncomment if needed
    usart_setup();
    i2c_setup();
    exti_setup();
    
    // *** CRITICAL CHANGE: Initialize as SNIFFER/LOGGER ***
    fusb_setup_sniffer(I2C1); 

    usart_printf("---- PD Packet Logger ----\\r\\n> ");

    char line[32];
    int pos = 0;

    while (1) {
        // Handle FUSB302 Interrupt
        if (fusb_event_pending) {
            fusb_handle_irq();
            fusb_event_pending = false;
        }

        // Handle UART CLI Non-Blocking (as in your original file)
        if (uart_rx_ready()) {
            char c = usart_recv(USART2);
            if (c=='\r' || c=='\n') {
                line[pos] = 0;
                usart_printf("\r\n");
                // handle_command(line); // Keep your existing command logic here
                pos = 0;
                usart_printf("> ");
            } 
            else if (pos < (int)sizeof(line)-1){
                // ... (your existing character handling) ...
            }
        }
    }
}

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
#include <stdbool.h> 

// Assuming fusb302.h contains necessary register and bit definitions
#include "fusb302.h"

// --- Global PD Definitions ---
#define MAX_PD_PACKET_SIZE 30 // Header (2) + Data Objects (7*4=28)
#define PD_MAX_PAYLOAD_SIZE 28 

// Buffer to store the raw packet data
uint8_t rx_buffer[MAX_PD_PACKET_SIZE];
// Status flag for exti handler
volatile bool fusb_event_pending = false;

// --- Custom PD Message Struct ---
typedef struct {
    uint16_t header;         
    uint8_t  msg_type;       
    uint8_t  data_role;      
    uint8_t  spec_rev;       
    uint8_t  power_role;     
    uint8_t  data_obj_count; 
    uint8_t  msg_id;         
    uint32_t obj[7]; // Data Objects
} pd_msg_t;

// --- Global Status Variables ---
static int g_pd_state = 0; // Simple integer for sniffer state
static uint8_t g_msg_id_counter = 0; // Not used for sniffer

/*---- HAL and Utility Function Prototypes (Assumed to be implemented) ----*/
extern uint8_t i2c_read_reg(uint8_t reg);
extern void i2c_write_reg(uint8_t reg, uint8_t val);
extern void fusb_read_fifo(uint8_t *data, size_t len);
extern int usart_printf(const char *format, ...);
extern bool uart_rx_ready(void);
extern char usart_recv(uint32_t usart);
extern void handle_command(char *line); // Existing CLI command handler

/*---- Core PD Utility Functions ----*/

// CRC-32 JAMCRC (Required for validating received packets)
uint32_t pd_calculate_crc(const uint8_t *data, size_t len)
{
    const uint32_t PD_CRC_POLY = 0xEDB88320; 
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x00000001) {
                crc = (crc >> 1) ^ PD_CRC_POLY;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc; 
}


/*---- Packet Logger Functions ----*/

/**
 * @brief Logs the contents of a PD message to the serial port.
 */
void pd_log_message(const pd_msg_t *msg)
{
    // Define message type strings for clean output
    const char *msg_type_data_str[] = {
        "Reserved", "Source_Caps", "Request", "BIST", "Sink_Caps", "Vendor_Def" 
    };
    const char *ctrl_msg_str[] = {
        "Reserved", "GoodCRC", "GotoMin", "Accept", "Reject", "Wait", 
        "PS_RDY", "Soft_Reset", "Not_Supported", "Get_Source_Cap", 
        "Get_Sink_Cap", "DR_Swap", "PR_Swap", "VCONN_Swap", "Send_DR_Swap",
        "Send_PR_Swap"
    };

    // Log Header Information
    usart_printf("[RX] %s%s ID:%u Rev:%u ", 
        PD_HEADER_POWER_ROLE(msg->header) ? "SRC" : "SNK",
        PD_HEADER_DATA_ROLE(msg->header) ? "/DFP" : "/UFP",
        msg->msg_id, msg->spec_rev);
    
    // Log Message Type
    if (msg->data_obj_count == 0) {
        // Control Message
        if (msg->msg_type < 16) {
            usart_printf("CTRL/%s\r\n", ctrl_msg_str[msg->msg_type]);
        } else {
            usart_printf("CTRL/Unknown(0x%02X)\r\n", msg->msg_type);
        }
    } else {
        // Data Message
        if (msg->msg_type < 6) {
             usart_printf("DATA/%s (%u DOs)\r\n", msg_type_data_str[msg->msg_type], msg->data_obj_count);
        } else {
            usart_printf("DATA/Unknown(0x%02X) (%u DOs)\r\n", msg->msg_type, msg->data_obj_count);
        }
        
        // Log data objects
        for (int i = 0; i < msg->data_obj_count; i++) {
            usart_printf("  DO%d: 0x%08X\r\n", i+1, msg->obj[i]);
        }
    }
}


/**
 * @brief Processes and validates a received PD packet from the FIFO.
 * @param msg Pointer to the structure to fill with parsed message data.
 * @return true if a valid packet was processed, false otherwise.
 */
bool pd_process_rx_fifo(pd_msg_t *msg)
{
    // 1. Read the first two bytes (Header) from the FIFO
    uint8_t header_bytes[2];
    fusb_read_fifo(header_bytes, 2); 
    msg->header = header_bytes[0] | (header_bytes[1] << 8);

    // 2. Extract length information
    msg->data_obj_count = PD_HEADER_NUM_DATA_OBJECTS(msg->header);
    int data_len = (msg->data_obj_count * 4) + 4; // Data(DOC*4) + CRC(4)
    if (data_len > PD_MAX_PAYLOAD_SIZE + 4) {
        i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_RX_FLUSH);
        return false;
    }

    // 3. Read remaining bytes (Data Objects and CRC)
    uint8_t rx_data_payload[PD_MAX_PAYLOAD_SIZE + 4];
    fusb_read_fifo(rx_data_payload, data_len);
    
    // 4. Prepare data for CRC check (Header + Data)
    uint8_t crc_check_buf[2 + data_len - 4]; 
    memcpy(crc_check_buf, header_bytes, 2);
    memcpy(crc_check_buf + 2, rx_data_payload, data_len - 4); 

    // 5. Check CRC
    uint32_t received_crc = rx_data_payload[data_len-4] | (rx_data_payload[data_len-3] << 8) |
                            (rx_data_payload[data_len-2] << 16) | (rx_data_payload[data_len-1] << 24);
    
    uint32_t calculated_crc = pd_calculate_crc(crc_check_buf, sizeof(crc_check_buf));

    // 6. Flush RX FIFO immediately
    i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_RX_FLUSH);

    if (received_crc != calculated_crc) return false; 

    // 7. Final Message Parsing
    msg->msg_type = PD_HEADER_MESSAGE_TYPE(msg->header);
    msg->data_role = PD_HEADER_DATA_ROLE(msg->header);
    msg->spec_rev = PD_HEADER_SPEC_REV(msg->header);
    msg->power_role = PD_HEADER_POWER_ROLE(msg->header);
    msg->msg_id = PD_HEADER_MESSAGE_ID(msg->header);
    
    // 8. Copy Data Objects
    for (int i = 0; i < msg->data_obj_count; i++) {
        msg->obj[i] = rx_data_payload[i*4] | (rx_data_payload[i*4+1] << 8) |
                      (rx_data_payload[i*4+2] << 16) | (rx_data_payload[i*4+3] << 24);
    }

    // CRITICAL STEP: Log the message immediately after successful parsing
    pd_log_message(msg);
    
    return true; 
}


/*---- FUSB302 Driver / Setup Functions ----*/

/**
 * @brief Configures the FUSB302 for Sniffer (Passive Listener) mode.
 * * This is the critical change from fusb_init_sink().
 */
void fusb_setup_sniffer(uint32_t i2c_periph)
{
    // 1. Full Reset
    i2c_write_reg(FUSB302_REG_RESET, FUSB302_RESET_SW_RES | FUSB302_RESET_PD_RES);
    
    // 2. Power up everything (except VCONN)
    i2c_write_reg(FUSB302_REG_POWER, FUSB302_PWR_ALL_MASK & ~FUSB302_PWR_VCONN_MASK); 
    
    // 3. Set Control Registers
    // CONTROL1: Enable SOP' packets (important for comprehensive sniffing)
    i2c_write_reg(FUSB302_REG_CONTROL1, FUSB302_CTL1_ENSOPC); 
    // CONTROL3: Disable auto-retry and auto-hard-reset
    i2c_write_reg(FUSB302_REG_CONTROL3, 0x00); 

    // 4. Set Switches to listen on both CC lines
    // TX on neither, Measure on both CC1 and CC2 (best for sniffer/debug)
    i2c_write_reg(FUSB302_REG_SWITCHES0, FUSB302_SW0_MEAS_CC1 | FUSB302_SW0_MEAS_CC2);
    // SWITCHES1: Ensure we are not auto-sending GoodCRC (FUSB302 defaults to ON, this is critical)
    i2c_write_reg(FUSB302_REG_SWITCHES1, 0x00); // Disables AUTO_GCRC if set

    // 5. Mask Interrupts
    // Unmask only the critical RX and Hard Reset interrupts
    i2c_write_reg(FUSB302_REG_MASKA, FUSB302_MASKA_ALL_MASK & ~(FUSB302_INTA_RX_STAT | FUSB302_INTA_HARDRST)); 
    // Unmask CRC_CHK for messages
    i2c_write_reg(FUSB302_REG_MASK, FUSB302_MASK_ALL_MASK & ~FUSB302_INT_CRC_CHK); 
    i2c_write_reg(FUSB302_REG_MASKC, FUSB302_MASKC_ALL_MASK); 
    
    // 6. Flush TX/RX FIFOs
    i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_RX_FLUSH | FUSB302_CTL0_TX_FLUSH);
    
    usart_printf("FUSB302 initialized for Passive Sniffer Mode.\r\n");
}


/**
 * @brief The main IRQ handler for the FUSB302.
 */
void fusb_handle_irq(void)
{
    // Reading these clears the interrupt flags
    uint8_t interrupt_a = i2c_read_reg(FUSB302_REG_INTERRUPTA);
    uint8_t interrupt_c = i2c_read_reg(FUSB302_REG_INTERRUPT); 
    
    // A. Handle Hard Reset Received (Resets the sniffer)
    if (interrupt_a & FUSB302_INTA_HARDRST) {
        usart_printf("\r\n--- HARD RESET ---\r\n");
        fusb_setup_sniffer(I2C1); 
    }
    
    // B. Handle RX Status/Message Received (CRC_CHK)
    if (interrupt_c & FUSB302_INT_CRC_CHK) {
        // Read, validate, and log the packet
        pd_msg_t rx_message;
        pd_process_rx_fifo(&rx_message);
    }
    
    // C. Handle TX/Retry Fail (Should not happen, but reset if it does)
    if (interrupt_a & FUSB302_INTA_RETRYFAIL) {
        usart_printf("\r\n--- TX Failure (unexpected) ---\r\n");
        fusb_setup_sniffer(I2C1); 
    }

    // D. Connection/Disconnect event (COMP_CHNG)
    if (interrupt_c & FUSB302_INT_COMP_CHNG) {
        // In sniffer mode, this indicates a cable connection/disconnection event.
        usart_printf("\r\n--- CC Status Change/Connection Event ---\r\n");
    }
}


/*---- Existing MCU Setup Functions (from your original pd_debug.c) ----*/

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
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

// NOTE: This printf function is assumed to be correct from your original file.
// int usart_printf(const char *format, ...) { ... } 

static void i2c_setup(void) {
    // I2C1 pins: SCL=PB6, SDA=PB7
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    
    i2c_reset(I2C1);
    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_sm_100k, 48); // Assuming 48MHz clock
    i2c_peripheral_enable(I2C1);
}

static void exti_setup(void) {
    // FUSB302 INT_N is on PB8 (Pin 44 of STM32F072)
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO8);
    
    exti_select_source(EXTI8, GPIOB);
    exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI8);
    
    nvic_enable_irq(NVIC_EXTI4_15_IRQ);
}

void exti4_15_isr(void) {
    if (exti_get_flag_status(EXTI8)) {
        fusb_event_pending = true;
        exti_clear_flag_status(EXTI8);
    }
}

// NOTE: The handle_pd_message() function is no longer needed 
// as logging is handled directly in pd_process_rx_fifo().

int main(void) {
    clock_setup();
    // systick_setup(); // Keep if needed for timing, commented out from your snippet
    usart_setup();
    i2c_setup();
    exti_setup();
    
    // *** CRITICAL CHANGE: Initialize as SNIFFER/LOGGER ***
    fusb_setup_sniffer(I2C1); 

    usart_printf("---- PD Packet Logger Initialized ----\\r\\n> ");

    char line[32];
    int pos = 0;

    while (1) {
        // Handle FUSB302 Interrupt (Non-blocking check)
        if (fusb_event_pending) {
            fusb_handle_irq();
            fusb_event_pending = false;
        }

        // Handle UART CLI
        if (uart_rx_ready()) {
            char c = usart_recv(USART2);
            if (c=='\r' || c=='\n') {
                line[pos] = 0;
                usart_printf("\r\n");
                handle_command(line); // Keep your existing CLI logic
                pos = 0;
                usart_printf("> ");
            } 
            else if (pos < (int)sizeof(line)-1){
                // ... (your existing character handling for line input) ...
            }
        }
    }
}