#include <stdint.h>
#include <stdbool.h>
#include <string.h> 
#include "fusb302.h"

// --- PD Message Data Structures ---

typedef union {
    uint32_t val;
    struct {
        uint8_t byte[4];
    } s;
} pd_data_obj_t;

typedef struct {
    uint16_t header;         
    uint8_t  msg_type;       
    uint8_t  data_role;      
    uint8_t  spec_rev;       
    uint8_t  power_role;
    uint8_t  data_obj_count; 
    uint8_t  msg_id;         
    pd_data_obj_t data[7]; 
} pd_msg_t;

// --- Policy Engine State Machine & Global Data ---

typedef enum {
    PE_INIT,
    PE_SRC_DISCOVERY,         // Initial state to detect connection
    PE_SRC_SEND_CAPABILITIES, // Sending Source_Capabilities
    PE_SRC_WAIT_REQUEST,      // Waiting for Request
    PE_SRC_TRANSITION_SUPPLY, // VBUS ramping up/down
    PE_SRC_READY,             // Contract established
} pe_state_t;

static pe_state_t g_pd_state = PE_INIT;
static pd_msg_t g_rx_message;
static uint8_t g_msg_id_counter = 0; // Global message ID counter (0-7)

// --- External HAL functions ---
extern uint8_t i2c_read_reg(uint8_t reg);
extern void i2c_write_reg(uint8_t reg, uint8_t val);
extern void fusb_read_fifo(uint8_t *data, size_t len);
extern uint32_t pd_calculate_crc(const uint8_t *data, size_t len);

// --- Helper function to construct a basic Source_Capabilities message ---
static void build_src_capabilities_msg(pd_msg_t *msg)
{
    // Use the global counter for the new message ID
    msg->msg_id = g_msg_id_counter; 
    
    // PD Spec Revision 3.0 (0x02)
    msg->spec_rev = 0x02; 
    // Data Role is DFP (0x01)
    msg->data_role = 0x01; 
    // Power Role is Source (0x01)
    msg->power_role = 0x01; 
    
    // Message Type: Source_Capabilities (Data Message Type 1)
    msg->msg_type = 0x01; 
    
    // Assume one 5V/3A Fixed Supply PDO
    msg->data_obj_count = 1;
    
    // Fixed Supply PDO: 
    // Bits 31-30=0 (Fixed), V=5000mV (10mV unit) -> 500 = 0x1F4
    // I=3000mA (10mA unit) -> 300 = 0x12C
    msg->data[0].val = (0 << 30) | (500 << 10) | (300); 
    msg->data[0].val |= (1 << 28); // Dual-Role Power (DRP) capable
}

bool pd_process_rx_fifo(pd_msg_t *msg)
{
    // The FUSB302 FIFO contains Header (2 bytes) + Data (N*4 bytes) + CRC (4 bytes).
    // The Policy Engine needs the full raw packet for CRC calculation (Header+Data+CRC).
    
    uint8_t header_bytes[2];
    uint8_t rx_buf[30]; // Max size: Header(2) + Data(7*4=28) = 30
    uint8_t *core_data = rx_buf;
    int data_len = 0;
    
    // Read the raw header bytes (Header is usually the first 2 bytes after tokens)
    // In the FUSB302, tokens are handled by the PHY. The FIFO contains the Header, Data, and CRC.
    // The safest way is to read the header and then use it to determine the length.
    fusb_read_fifo(header_bytes, 2); 
    msg->header = header_bytes[0] | (header_bytes[1] << 8);

    // Extract Data Object Count (DOC) from the header
    msg->data_obj_count = (msg->header >> 12) & 0x07; // Bits 14:12
    
    // Calculate remaining bytes to read (Data + CRC)
    data_len = (msg->data_obj_count * 4) + 4; // Data(DOC*4) + CRC(4)
    if (data_len > sizeof(rx_buf)) return false; // Max data objects exceeded

    // Read remaining bytes (Data Objects and CRC)
    fusb_read_fifo(rx_buf, data_len);
    
    // Prepare the data for CRC check (Header + Data)
    uint8_t crc_check_buf[30]; // Header(2) + Data(N*4), max 30 bytes
    memcpy(crc_check_buf, header_bytes, 2);
    memcpy(crc_check_buf + 2, rx_buf, data_len - 4); // Copy Data Objects

    // Check CRC (CRC is the last 4 bytes of the data read)
    uint32_t received_crc = rx_buf[data_len-4] | (rx_buf[data_len-3] << 8) |
                            (rx_buf[data_len-2] << 16) | (rx_buf[data_len-1] << 24);
    
    uint32_t calculated_crc = pd_calculate_crc(crc_check_buf, sizeof(crc_check_buf));

    // Flush RX FIFO immediately
    i2c_write_reg(FUSB302_REG_CONTROL1, FUSB302_CTL1_RX_FLUSH);

    if (received_crc != calculated_crc) return false; 

    // Final Message Parsing
    msg->msg_type = (msg->header) & 0x1F;            // Bits 4:0
    msg->data_role = (msg->header >> 5) & 0x01;      // Bit 5
    msg->spec_rev = (msg->header >> 6) & 0x03;       // Bits 6-7
    msg->power_role = (msg->header >> 8) & 0x01;     // Bit 8
    msg->msg_id = (msg->header >> 9) & 0x07;         // Bits 9-11
    
    // Copy Data Objects
    for (int i = 0; i < msg->data_obj_count; i++) {
        memcpy(msg->data[i].s.byte, &rx_buf[i * 4], 4);
    }

    return true; 
}

bool pd_send_message(const pd_msg_t *msg)
{
    // Flush the TX FIFO
    i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_FLUSH);

    // Prepare the packet
    uint8_t tx_buf[32]; 
    int index = 0;
    
    // Data to be CRC'd (Header + Data)
    uint8_t crc_data[30];
    int crc_index = 0;

    // Start-of-Packet Token (SOP1 for FUSB302)
    tx_buf[index++] = FUSB302_TX_TKN_SOP1; 

    // PD Header (16 bits, LSB first)
    // Reconstruct the header using all fields including Power Role (bit 8)
    uint16_t header = msg->msg_type 
                      | (0x00 << 4) // Reserved
                      | (msg->data_role << 5)
                      | (msg->spec_rev << 6)
                      | (msg->power_role << 8) // Power Role (Source/Sink)
                      | (g_msg_id_counter << 9) // Use global message ID
                      | (msg->data_obj_count << 12);
    
    // Store header in CRC buffer
    crc_data[crc_index++] = (uint8_t)(header & 0xFF);     
    crc_data[crc_index++] = (uint8_t)(header >> 8);       
    
    // Write PACKSYM token + Header Length (2 bytes)
    tx_buf[index++] = FUSB302_TX_TKN_PACKSYM | 2; 
    tx_buf[index++] = (uint8_t)(header & 0xFF);
    tx_buf[index++] = (uint8_t)(header >> 8);
    
    // Data Objects (4 bytes each)
    if (msg->data_obj_count > 0) {
        // Write PACKSYM token + Data Length (DOC * 4 bytes)
        tx_buf[index++] = FUSB302_TX_TKN_PACKSYM | (msg->data_obj_count * 4); 
        for (int i = 0; i < msg->data_obj_count; i++) {
            // Write data to TX buffer
            memcpy(&tx_buf[index], msg->data[i].s.byte, 4);
            index += 4;
            // Copy data to CRC buffer
            memcpy(&crc_data[crc_index], msg->data[i].s.byte, 4);
            crc_index += 4;
        }
    }

    // Calculate 32-bit JAMCRC over Header + Data
    uint32_t crc = pd_calculate_crc(crc_data, crc_index); 
    
    // Write JAMCRC Token
    tx_buf[index++] = FUSB302_TX_TKN_JAMCRC; 

    // End-of-Packet Token (EOP)
    tx_buf[index++] = FUSB302_TX_TKN_EOP; 

    // TX_OFF Token (Signal end of stream)
    tx_buf[index++] = FUSB302_TX_TKN_TXOFF; 

    // Write the entire buffer to the FIFOS register
    for (int i = 0; i < index; i++) {
        i2c_write_reg(FUSB302_REG_FIFOS, tx_buf[i]);
    }

    // Start transmission (write to CONTROL0 to start TX)
    i2c_write_reg(FUSB302_REG_CONTROL0, FUSB302_CTL0_TX_START); 
    
    // Increment global message ID counter
    g_msg_id_counter = (g_msg_id_counter + 1) & 0x07;

    return true;
}

// State machine
void fusb302_handle_irq(void)
{
    // Read the FUSB302 interrupt status registers 
    uint8_t interrupt_a = i2c_read_reg(FUSB302_REG_INTERRUPTA);
    uint8_t interrupt_c = i2c_read_reg(FUSB302_REG_INTERRUPT); // Use INTERRUPT (0x42) for COMP_CHNG
    uint8_t status0 = i2c_read_reg(FUSB302_REG_STATUS0);
    
    // FUSB302 interrupts are generally cleared by reading, but we must explicitly 
    // address all flags to ensure the policy engine responds correctly.
    
    // Handle CC Detection Change (Start of connection)
    if (interrupt_c & FUSB302_INT_COMP_CHNG) {
        
        // This usually signals a change in the CC pin comparator status.
        // We must check STATUS0 and SWITCHES registers to see if we transitioned 
        // from 'Open' to 'Rd' (Sink attached).
        
        if (g_pd_state == PE_INIT || g_pd_state == PE_SRC_DISCOVERY) {
            
            // Assume initial negotiation starts on COMP_CHNG if we are a Source
            // and the comparator detects an Rd (Sink). 
            // The logic here is highly specific to CC detection/role.
            
            // Simplification: Assume COMP_CHNG means a device is attached, 
            // and we should proceed to send capabilities.
            
            g_pd_state = PE_SRC_DISCOVERY; // Acknowledge attachment
            
            // Immediately transition to sending capabilities
            // Check status0 to confirm CC line is active
            if (status0 & FUSB302_STATUS0_COMP) { 
                
                // Transition to the capability sending state.
                pd_msg_t cap_msg;
                build_src_capabilities_msg(&cap_msg); 
                
                if (pd_send_message(&cap_msg)) {
                    g_pd_state = PE_SRC_WAIT_REQUEST; 
                }
            }
        }
    }

    // Handle RX Status/Message Received (CRC_CHK in INTERRUPT (0x42))
    if (interrupt_c & FUSB302_INT_CRC_CHK) {
        
        // FUSB302 automatically sends GoodCRC, the interrupt means a valid message 
        // (Header + Data + CRC) has been received and is in the FIFO.
        
        if (pd_process_rx_fifo(&g_rx_message)) {
            
            uint8_t rx_msg_type = g_rx_message.msg_type;

            switch (g_pd_state) {
                
                case PE_SRC_WAIT_REQUEST:
                    // Waiting for Request (Data Message Type 2)
                    if (rx_msg_type == 0x02) { 
                        uint8_t response_id = g_rx_message.msg_id;
                        
                        // TODO: Implement Request validation logic here.
                        bool request_is_valid = true; 

                        if (request_is_valid) { 
                            // Send ACCEPT (Control Message Type 3)
                            pd_msg_t accept_msg = { 
                                .msg_type = 0x03, 
                                .data_obj_count = 0, 
                                .msg_id = response_id, // Match received ID
                                .spec_rev = 0x02,
                                .data_role = 0x01,
                                .power_role = 0x01
                            };
                            pd_send_message(&accept_msg);
                            
                            // NOTE: VBUS change logic needed here.
                            
                            g_pd_state = PE_SRC_TRANSITION_SUPPLY;
                        } else {
                            // Send REJECT (Type 4) or WAIT (Type 5)
                        }
                    }
                    break;

                case PE_SRC_TRANSITION_SUPPLY:
                    // Wait for PS_RDY (Control Message Type 6) after VBUS change
                    if (rx_msg_type == 0x06) { 
                        g_pd_state = PE_SRC_READY;
                    }
                    break;
                
                case PE_SRC_READY:
                    // Handle messages in steady state (e.g., Get_Sink_Cap)
                    if (rx_msg_type == 0x0A) { // PD_CTRL_GET_SINK_CAP
                        // TODO: Respond with Sink_Capabilities (if we were DRP)
                    }
                    break;
                
                default:
                    // Handle unexpected message in current state (e.g., request in READY state)
                    break;
            }
        } 
    }
    
    // Handle Hard Reset Received
    if (interrupt_a & FUSB302_INTA_HARDRST) {
        // Clear Hard Reset bit (usually done by reading, but explicitly for clarity)
        // Set g_pd_state to initial state
        g_pd_state = PE_INIT; 
    }
    
    // Handle TX Failure/Retry Fail
    if (interrupt_a & (FUSB302_INTA_RETRYFAIL)) {
        // FUSB302 attempts 3 retries automatically. If this fires, the message failed.
        // Initiate Hard Reset or retry negotiation.
        
        // Send a Hard Reset
        i2c_write_reg(FUSB302_REG_CONTROL3, FUSB302_CTL3_SEND_HARD_RESET); 
        g_pd_state = PE_INIT;
    }
}