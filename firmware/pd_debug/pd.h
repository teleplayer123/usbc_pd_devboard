#include <stdint.h>
#include <stdlib.h>

/* -----------------------------------------------------------
 * PD Macros/Defines
 * ----------------------------------------------------------- */

// Maximum length of a standard PD Data Message (Header + 7 PDOs + CRC)
#define MAX_PD_PACKET_SIZE (2 + 7*4 + 4) // Header + 7 Data Objects + CRC = 34 bytes

/*
* @brief Construct a PD message header
* @param type: 5 bit Message Type 										   			   			[Bit 4:0]	
* @param prole: 1 bit Port Power Role (0 = Sink, 1 = Source)		   				   			[Bit 8]	
* @param drole: 1 bit Device Role (0 = UFP, 1 = DFP)							       			[Bit 5]
* @param id: 3 bit Message ID initialized to 0 at power on as result of SoftReset or HardReset  [Bit 11:9]	
*			 ID is incremented by 1 when message successfully received indicated by GoodCRC
* @param cnt: 3 bit Number of Data Objects (0-7)									   			[Bit 14:12]
* @param rev: 2 bit Specification Revision											   			[Bit 7:6]
* @param ext: 1 bit Extended Message Indicator (0 = Control Message, 1 = Data Message) 			[Bit 15]
*/
#define PD_HEADER(type, prole, drole, id, cnt, rev, ext) \
	((type) | ((rev) << 6) | \
	((drole) << 5) | ((prole) << 8) | \
	((id) << 9) | ((cnt) << 12) | ((ext) << 15))

// Process PD headers
#define PD_HEADER_EXT(h)  (((h) >> 15) & 1)
#define PD_HEADER_CNT(h)  (((h) >> 12) & 7)
#define PD_HEADER_TYPE(h) ((h) & 0x1F)
#define PD_HEADER_ID(h)   (((h) >> 9) & 7)
#define PD_HEADER_REV(h)  (((h) >> 6) & 3)

#define PD_SRC_DEF_MV               1600
#define PD_SRC_DEF_RD_MV            200
#define PD_RETRY_COUNT              3

// Port Power Role
#define PD_POWER_ROLE_SINK    0
#define PD_POWER_ROLE_SOURCE  1

// Port Data Role
#define PD_DATA_ROLE_UFP 0
#define PD_DATA_ROLE_DFP 1

// Specification Revision
#define PD_SPEC_REV1  0 // deprecated
#define PD_SPEC_REV2  1
#define PD_SPEC_REV3  2

#define ARRAY_SIZE(t) (sizeof(t) / sizeof(t[0]))

#define PDO_FIXED_DUAL_ROLE (1 << 29) // Dual role device
#define PDO_FIXED_SUSPEND   (1 << 28) // USB Suspend supported
#define PDO_FIXED_EXTERNAL  (1 << 27) // Externally powered 
#define PDO_FIXED_COMM_CAP  (1 << 26) // USB Communications Capable
#define PDO_FIXED_DATA_SWAP (1 << 25) // Data role swap command supported
#define PDO_FIXED_PEAK_CURR () // Peak current 
#define PDO_FIXED_VOLT(mv)  (((mv)/50) << 10) // Voltage in 50mV units
#define PDO_FIXED_CURR(ma)  (((ma)/10) << 0)  // Max current in 10mA units

#define PDO_FIXED(mv, ma, flags) (PDO_FIXED_VOLT(mv) |\
				  PDO_FIXED_CURR(ma) | (flags))

#define PDO_FIXED_FLAGS (PDO_FIXED_DUAL_ROLE | PDO_FIXED_DATA_SWAP |\
PDO_FIXED_COMM_CAP)

const uint32_t pd_src_pdo[] = {
	PDO_FIXED(5000, 1500, PDO_FIXED_FLAGS),
};
const int pd_src_pdo_cnt = ARRAY_SIZE(pd_src_pdo);

const uint32_t pd_snk_pdo[] = {
	PDO_FIXED(5000, 500, PDO_FIXED_FLAGS),
	PDO_FIXED(9000, 500, PDO_FIXED_FLAGS),
	PDO_FIXED(20000, 500, PDO_FIXED_FLAGS),
};
const int pd_snk_pdo_cnt = ARRAY_SIZE(pd_snk_pdo);

/* Control Message type - USB-PD Spec Rev 3.2, Ver 1.1, Table 6-5 */
enum pd_ctrl_msg_type {
	PD_CTRL_INVALID = 0, // 0 Reserved - DO NOT PUT IN MESSAGES
	PD_CTRL_GOOD_CRC = 1,
	PD_CTRL_GOTO_MIN = 2, // Deprecated
	PD_CTRL_ACCEPT = 3,
	PD_CTRL_REJECT = 4,
	PD_CTRL_PING = 5,	// Deprecated
	PD_CTRL_PS_RDY = 6,
	PD_CTRL_GET_SOURCE_CAP = 7,
	PD_CTRL_GET_SINK_CAP = 8,
	PD_CTRL_DR_SWAP = 9,
	PD_CTRL_PR_SWAP = 10,
	PD_CTRL_VCONN_SWAP = 11,
	PD_CTRL_WAIT = 12,
	PD_CTRL_SOFT_RESET = 13,
	/* Used for REV 3.0 */
	PD_CTRL_DATA_RESET = 14,
	PD_CTRL_DATA_RESET_COMPLETE = 15,
	PD_CTRL_NOT_SUPPORTED = 16,
	PD_CTRL_GET_SOURCE_CAP_EXT = 17,
	PD_CTRL_GET_STATUS = 18,
	PD_CTRL_FR_SWAP = 19,
	PD_CTRL_GET_PPS_STATUS = 20,
	PD_CTRL_GET_COUNTRY_CODES = 21,
	PD_CTRL_GET_SINK_CAP_EXT = 22,
	/* Used for REV 3.1 */
	PD_CTRL_GET_SOURCE_INFO = 23,
	PD_CTRL_GET_REVISION = 24,
	/* 25-31 Reserved */
};

enum pd_data_msg_type {
	PD_DATA_INVALID = 0, /* 0 Reserved - DO NOT PUT IN MESSAGES */
	PD_DATA_SOURCE_CAPABILITIES = 1,
	PD_DATA_REQUEST = 2,
	PD_DATA_BIST = 3,
	PD_DATA_SINK_CAPABILITIES = 4,
	PD_DATA_BATTERY_STATUS = 5,
	PD_DATA_ALERT = 6,
	PD_DATA_GET_COUNTRY_INFO = 7,
	PD_DATA_ENTER_USB = 8,
	PD_DATA_EPR_REQUEST = 9,
	PD_DATA_EPR_MODE = 10,
	PD_DATA_SOURCE_INFO = 11,
	PD_DATA_REVISION = 12,
	/* 13-14 Reserved */
	PD_DATA_VENDOR_DEFINED = 15,
	/* 16-31 Reserved */
};

enum tcpc_cc_voltage_status {
	TYPEC_CC_VOLT_OPEN = 0,
	TYPEC_CC_VOLT_RA = 1,
	TYPEC_CC_VOLT_RD = 2,
	TYPEC_CC_VOLT_SNK_DEF = 5,
	TYPEC_CC_VOLT_SNK_1_5 = 6,
	TYPEC_CC_VOLT_SNK_3_0 = 7,
};

enum tcpc_cc_pull {
	TYPEC_CC_RA = 0,
	TYPEC_CC_RP = 1,
	TYPEC_CC_RD = 2,
	TYPEC_CC_OPEN = 3,
};

enum tcpc_message_type {
    TYPEC_MESSAGE_TYPE_SOP = 0,
    TYPEC_MESSAGE_TYPE_SOP_PRIME = 1,
    TYPEC_MESSAGE_TYPE_SOP_DOUBLE_PRIME = 2,
    TYPEC_MESSAGE_TYPE_SOP_DBG_PRIME = 3,
    TYPEC_MESSAGE_TYPE_SOP_DBG_DOUBLE_PRIME = 4,
    TYPEC_MESSAGE_TYPE_HARD_RESET = 5,
    TYPEC_MESSAGE_TYPE_CABLE_RESET = 6,
    TYPEC_MESSAGE_TYPE_BIST_MODE_2 = 7,
};