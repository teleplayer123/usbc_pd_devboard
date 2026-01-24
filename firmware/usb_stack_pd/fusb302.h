#pragma once
#ifndef FUSB302_H
#define FUSB302_H

#include <stdint.h>
#include <stdlib.h>

/* -----------------------------------------------------------
 * I2C Address
 * ----------------------------------------------------------- */

#define FUSB302_ADDR            0x22

/* -----------------------------------------------------------
 * Register Addresses
 * ----------------------------------------------------------- */
#define FUSB302_REG_DEVICE_ID       0x01
#define FUSB302_REG_SWITCHES0       0x02
#define FUSB302_REG_SWITCHES1       0x03
#define FUSB302_REG_MEASURE         0x04
#define FUSB302_REG_SLICE           0x05
#define FUSB302_REG_CONTROL0        0x06
#define FUSB302_REG_CONTROL1        0x07
#define FUSB302_REG_CONTROL2        0x08
#define FUSB302_REG_CONTROL3        0x09
#define FUSB302_REG_MASK            0x0A
#define FUSB302_REG_POWER           0x0B
#define FUSB302_REG_RESET           0x0C
#define FUSB302_REG_OCPREG          0x0D
#define FUSB302_REG_MASKA           0x0E
#define FUSB302_REG_MASKB           0x0F
#define FUSB302_REG_CONTROL4        0x10

#define FUSB302_REG_STATUS0A        0x3C
#define FUSB302_REG_STATUS1A        0x3D
#define FUSB302_REG_INTERRUPTA      0x3E
#define FUSB302_REG_INTERRUPTB      0x3F

#define FUSB302_REG_STATUS0         0x40
#define FUSB302_REG_STATUS1         0x41
#define FUSB302_REG_INTERRUPT       0x42
#define FUSB302_REG_FIFOS           0x43

/* -----------------------------------------------------------
 * SWITCHES0 (0x02)
 * ----------------------------------------------------------- */
#define FUSB302_SW0_PU_EN2          (1 << 7)
#define FUSB302_SW0_PU_EN1          (1 << 6)
#define FUSB302_SW0_VCONN_CC2       (1 << 5)
#define FUSB302_SW0_VCONN_CC1       (1 << 4)
#define FUSB302_SW0_MEAS_CC2        (1 << 3)
#define FUSB302_SW0_MEAS_CC1        (1 << 2)
#define FUSB302_SW0_PDWN2           (1 << 1)
#define FUSB302_SW0_PDWN1           (1 << 0)

/* -----------------------------------------------------------
 * SWITCHES1 (0x03)
 * ----------------------------------------------------------- */
#define FUSB302_SW1_POWERROLE       (1 << 7)
#define FUSB302_SW1_SPECREV1        (1 << 6)
#define FUSB302_SW1_SPECREV0        (1 << 5)
#define FUSB302_SW1_DATAROLE        (1 << 4)
#define FUSB302_SW1_AUTO_GCRC       (1 << 2)
#define FUSB302_SW1_TXCC2           (1 << 1)
#define FUSB302_SW1_TXCC1           (1 << 0)

/* -----------------------------------------------------------
 * MEASURE (0x04)
 * ----------------------------------------------------------- */
#define FUSB302_MEAS_VBUS           (1 << 6)
#define FUSB302_MEAS_MDAC_MASK      0x3F
// Convert voltage to MDAC code for programming the fusb302
#define DIV_ROUND_NEAREST(x, y) (((x) + ((y) / 2)) / (y))
#define FUSB302_MEAS_MDAC_MV(mv)    (DIV_ROUND_NEAREST((mv), 42) & 0x3F) // MDAC step = 42mV

/* -----------------------------------------------------------
 * SLICE (0x05)
 * ----------------------------------------------------------- */
#define FUSB302_SLICE_SDAC_HYS_POS  6
#define FUSB302_SLICE_SDAC_HYS_MASK (3 << FUSB302_SLICE_SDAC_HYS_POS)
#define FUSB302_SLICE_SDAC_POS      0
#define FUSB302_SLICE_SDAC_MASK     (0x3F << FUSB302_SLICE_SDAC_POS)

/* -----------------------------------------------------------
 * CONTROL0 (0x06)
 * ----------------------------------------------------------- */
#define FUSB302_CTL0_TX_FLUSH       (1 << 6)
#define FUSB302_CTL0_INT_MASK       (1 << 5)
#define FUSB302_CTL0_HOST_CUR_POS   2
#define FUSB302_CTL0_HOST_CUR_MASK  (3 << FUSB302_CTL0_HOST_CUR_POS)
#define FUSB302_CTL0_HOST_CUR_3A0   (3 << FUSB302_CTL0_HOST_CUR_POS)
#define FUSB302_CTL0_HOST_CUR_1A5   (2 << FUSB302_CTL0_HOST_CUR_POS)
#define FUSB302_CTL0_HOST_CUR_USB   (1 << FUSB302_CTL0_HOST_CUR_POS)
#define FUSB302_CTL0_HOST_CUR1      (1 << 3)
#define FUSB302_CTL0_HOST_CUR0      (1 << 2)
#define FUSB302_CTL0_AUTO_PRE       (1 << 1)
#define FUSB302_CTL0_TX_START       (1 << 0)

/* -----------------------------------------------------------
 * CONTROL1 (0x07)
 * ----------------------------------------------------------- */
#define FUSB302_CTL1_ENSOP2DB       (1 << 6)
#define FUSB302_CTL1_ENSOP1DB       (1 << 5)
#define FUSB302_CTL1_BIST_MODE2     (1 << 4)
#define FUSB302_CTL1_RX_FLUSH       (1 << 2)
#define FUSB302_CTL1_ENSOP2         (1 << 1)
#define FUSB302_CTL1_ENSOP1         (1 << 0)

/* -----------------------------------------------------------
 * CONTROL2 (0x08)
 * ----------------------------------------------------------- */
#define FUSB302_CTL2_TOG_SAVE_PWR2  (1 << 7)
#define FUSB302_CTL2_TOG_SAVE_PWR1  (1 << 6)
#define FUSB302_CTL2_TOG_RD_ONLY    (1 << 5)
#define FUSB302_CTL2_WAKE_EN        (1 << 3)
#define FUSB302_CTL2_MODE_POS       1
#define FUSB302_CTL2_MODE_MASK      (3 << FUSB302_CTL2_MODE_POS)
#define FUSB302_CTL2_MODE_1         (1 << 2)
#define FUSB302_CTL2_MODE_0         (1 << 1)
#define FUSB302_CTL2_TOGGLE         (1 << 0)

#define FUSB302_CTL2_MODE_UFP       (0 << FUSB302_CTL2_MODE_POS)
#define FUSB302_CTL2_MODE_DFP       (1 << FUSB302_CTL2_MODE_POS)
#define FUSB302_CTL2_MODE_DRP       (2 << FUSB302_CTL2_MODE_POS)

/* -----------------------------------------------------------
 * CONTROL3 (0x09)
 * ----------------------------------------------------------- */
#define FUSB302_CTL3_SEND_HARDRESET (1 << 6)
#define FUSB302_CTL3_BIST_TMODE     (1 << 5)
#define FUSB302_CTL3_AUTO_HARDRESET (1 << 4)
#define FUSB302_CTL3_AUTO_SOFTRESET (1 << 3)
#define FUSB302_CTL3_NRETRIES_POS   1
#define FUSB302_CTL3_NRETRIES_MASK  (3 << FUSB302_CTL3_NRETRIES_POS)
#define FUSB302_CTL3_NRETRIES1      (1 << 2)
#define FUSB302_CTL3_NRETRIES0      (1 << 1)
#define FUSB302_CTL3_AUTO_RETRY     (1 << 0)

/* -----------------------------------------------------------
 * MASK (0x0A)
 * ----------------------------------------------------------- */
#define FUSB302_MASK_VBUSOK         (1 << 7)
#define FUSB302_MASK_ACTIVITY       (1 << 6)
#define FUSB302_MASK_COMP_CHNG      (1 << 5)
#define FUSB302_MASK_CRC_CHK        (1 << 4)
#define FUSB302_MASK_ALERT          (1 << 3)
#define FUSB302_MASK_WAKE           (1 << 2)
#define FUSB302_MASK_COLLISION      (1 << 1)
#define FUSB302_MASK_BC_LVL         (1 << 0)

/* -----------------------------------------------------------
 * POWER (0x0B)
 * ----------------------------------------------------------- */
#define FUSB302_POWER_BANDGAP       (1 << 0)
#define FUSB302_POWER_RX_REF        (1 << 1)
#define FUSB302_POWER_MEAS_BLOCK    (1 << 2)
#define FUSB302_POWER_INTERNAL_OSC  (1 << 3)
#define FUSB302_POWER_ALL_ON        (FUSB302_POWER_BANDGAP | FUSB302_POWER_RX_REF | FUSB302_POWER_MEAS_BLOCK | FUSB302_POWER_INTERNAL_OSC)

/* -----------------------------------------------------------
 * RESET (0x0C)
 * ----------------------------------------------------------- */
#define FUSB302_RESET_PD            (1 << 1)
#define FUSB302_RESET_SW            (1 << 0)

/* -----------------------------------------------------------
 * OCPREG (0x0D)
 * ----------------------------------------------------------- */
#define FUSB302_OCP_RANGE           (1 << 3)
#define FUSB302_OCP_CUR_POS         0
#define FUSB302_OCP_CUR_MASK        (7 << FUSB302_OCP_CUR_POS)

/* -----------------------------------------------------------
 * MASKA (0x0E)
 * ----------------------------------------------------------- */
#define FUSB302_MASKA_OCP_TEMP      (1 << 7)
#define FUSB302_MASKA_TOGDONE       (1 << 6)
#define FUSB302_MASKA_SOFTFAIL      (1 << 5)
#define FUSB302_MASKA_RETRYFAIL     (1 << 4)
#define FUSB302_MASKA_HARDSENT      (1 << 3)
#define FUSB302_MASKA_TXSENT        (1 << 2)
#define FUSB302_MASKA_SOFTRST       (1 << 1)
#define FUSB302_MASKA_HARDRST       (1 << 0)

/* -----------------------------------------------------------
 * MASKB (0x0F)
 * ----------------------------------------------------------- */
#define FUSB302_MASKB_GCRCSENT      (1 << 0)

/* -----------------------------------------------------------
 * CONTROL4 (0x10)
 * ----------------------------------------------------------- */
#define FUSB302_CTL4_TOG_EXIT_AUD   (1 << 0)

/* -----------------------------------------------------------
 * STATUS0A (0x3C)
 * ----------------------------------------------------------- */
#define FUSB302_STATUS0A_SOFTFAIL   (1 << 5)
#define FUSB302_STATUS0A_RETRYFAIL  (1 << 4)
#define FUSB302_STATUS0A_POWER3     (1 << 3)
#define FUSB302_STATUS0A_POWER2     (1 << 2)
#define FUSB302_STATUS0A_SOFTRST    (1 << 1)
#define FUSB302_STATUS0A_HARDRST    (1 << 0)

/* -----------------------------------------------------------
 * STATUS1A (0x3D)
 * ----------------------------------------------------------- */
#define FUSB302_STATUS1A_TOGSS3     (1 << 5)
#define FUSB302_STATUS1A_TOGSS2     (1 << 4)
#define FUSB302_STATUS1A_TOGSS1     (1 << 3)
#define FUSB302_STATUS1A_RXSOP2DB   (1 << 2)
#define FUSB302_STATUS1A_RXSOP1DB   (1 << 1)
#define FUSB302_STATUS1A_RXSOP      (1 << 0)

/* -----------------------------------------------------------
 * INTERRUPTA (0x3E)
 * ----------------------------------------------------------- */
#define FUSB302_INTA_OCP_TEMP       (1 << 7)
#define FUSB302_INTA_TOGDONE        (1 << 6)
#define FUSB302_INTA_SOFTFAIL       (1 << 5)
#define FUSB302_INTA_RETRYFAIL      (1 << 4)
#define FUSB302_INTA_HARDSENT       (1 << 3)
#define FUSB302_INTA_TXSENT         (1 << 2)
#define FUSB302_INTA_SOFTRST        (1 << 1)
#define FUSB302_INTA_HARDRST        (1 << 0)

/* -----------------------------------------------------------
 * INTERRUPTB (0x3F)
 * ----------------------------------------------------------- */
#define FUSB302_INTB_GCRCSENT       (1 << 0)

/* -----------------------------------------------------------
 * STATUS0 (0x40)
 * ----------------------------------------------------------- */
#define FUSB302_STATUS0_VBUSOK      (1 << 7)
#define FUSB302_STATUS0_ACTIVITY    (1 << 6)
#define FUSB302_STATUS0_COMP        (1 << 5)
#define FUSB302_STATUS0_CRC_CHK     (1 << 4)
#define FUSB302_STATUS0_ALERT       (1 << 3)
#define FUSB302_STATUS0_WAKE        (1 << 2)
#define FUSB302_STATUS0_BC_LVL1     (1 << 1)
#define FUSB302_STATUS0_BC_LVL0     (1 << 0)
#define FUSB302_STATUS0_BC_LVL_POS  0
#define FUSB302_STATUS0_BC_LVL_MASK (3 << FUSB302_STATUS0_BC_LVL_POS)

/* -----------------------------------------------------------
 * STATUS1 (0x41)
 * ----------------------------------------------------------- */
#define FUSB302_STATUS1_RXSOP2      (1 << 7)
#define FUSB302_STATUS1_RXSOP1      (1 << 6)
#define FUSB302_STATUS1_RX_EMPTY    (1 << 5)
#define FUSB302_STATUS1_RX_FULL     (1 << 4)
#define FUSB302_STATUS1_TX_EMPTY    (1 << 3)
#define FUSB302_STATUS1_TX_FULL     (1 << 2)
#define FUSB302_STATUS1_OVRTEMP     (1 << 1)
#define FUSB302_STATUS1_OCP         (1 << 0)

/* -----------------------------------------------------------
 * INTERRUPT (0x42)
 * ----------------------------------------------------------- */
#define FUSB302_INT_VBUSOK          (1 << 7)
#define FUSB302_INT_ACTIVITY        (1 << 6)
#define FUSB302_INT_COMP_CHNG       (1 << 5)
#define FUSB302_INT_CRC_CHK         (1 << 4)
#define FUSB302_INT_ALERT           (1 << 3)
#define FUSB302_INT_WAKE            (1 << 2)
#define FUSB302_INT_COLLISION       (1 << 1)
#define FUSB302_INT_BC_LVL          (1 << 0)

// Maximum size of the RX FIFO
#define FUSB302_RX_FIFO_SIZE 80

#define BIT(x) (1 << (x))

/* -----------------------------------------------------------
 * Tokens for FIFOS register
 * ----------------------------------------------------------- */

enum fusb302_fifo_tokens {
    FUSB302_TKN_TXON = 0xA1,
    FUSB302_TKN_SOP1 = 0x12,
    FUSB302_TKN_SOP2 = 0x13,
    FUSB302_TKN_SOP3 = 0x1B,
    FUSB302_TKN_RESET1 = 0x15,
    FUSB302_TKN_RESET2 = 0x16,
    FUSB302_TKN_PACKSYM = 0x80,
    FUSB302_TKN_JAMCRC = 0xFF,
    FUSB302_TKN_EOP = 0x14,
    FUSB302_TKN_TXOFF = 0xFE,
};

enum fusb302_fifo_rx_tokens {
    FUSB302_RX_TKN_SOP = 0xE0,
    FUSB302_RX_TKN_SOP1 = 0xC0,
    FUSB302_RX_TKN_SOP2 = 0xA0,
    FUSB302_RX_TKN_SOP1DB = 0x80,
    FUSB302_RX_TKN_SOP2DB = 0x60,
};

/* -----------------------------------------------------------
 * Register structs for debugging (printing bits)
 * ----------------------------------------------------------- */

 struct bit_name {
    int mask;
    const char *name;
};

/*------------ Status Registers ------------*/

static const struct bit_name fusb302_status0_bits[] = {
    {FUSB302_STATUS0_VBUSOK, "VBUSOK"},
    {FUSB302_STATUS0_ACTIVITY, "ACTIVITY"},
    {FUSB302_STATUS0_COMP, "COMP"},
    {FUSB302_STATUS0_CRC_CHK, "CRC_CHK"},
    {FUSB302_STATUS0_ALERT, "ALERT"},
    {FUSB302_STATUS0_WAKE, "WAKE"},
    {FUSB302_STATUS0_BC_LVL1, "BC_LVL1"},
    {FUSB302_STATUS0_BC_LVL0, "BC_LVL0"},
};

static const struct bit_name fusb302_status1_bits[] = {
    {FUSB302_STATUS1_RXSOP2, "RXSOP2"},
    {FUSB302_STATUS1_RXSOP1, "RXSOP1"},
    {FUSB302_STATUS1_RX_EMPTY, "RX_EMPTY"},
    {FUSB302_STATUS1_RX_FULL, "RX_FULL"},
    {FUSB302_STATUS1_TX_EMPTY, "TX_EMPTY"},
    {FUSB302_STATUS1_TX_FULL, "TX_FULL"},
    {FUSB302_STATUS1_OVRTEMP, "OVRTEMP"},
    {FUSB302_STATUS1_OCP, "OCP"},
};

static const struct bit_name fusb302_status0a_bits[] = {
    {BIT(7), NULL},
    {BIT(6), NULL},
    {FUSB302_STATUS0A_SOFTFAIL, "SOFTFAIL"},
    {FUSB302_STATUS0A_RETRYFAIL, "RETRYFAIL"},
    {FUSB302_STATUS0A_POWER3, "POWER3"},
    {FUSB302_STATUS0A_POWER2, "POWER2"},
    {FUSB302_STATUS0A_SOFTRST, "SOFTRST"},
    {FUSB302_STATUS0A_HARDRST, "HARDRST"},
};

static const struct bit_name fusb302_status1a_bits[] = {
    {BIT(7), NULL},
    {BIT(6), NULL},
    {FUSB302_STATUS1A_TOGSS3, "TOGSS3"},
    {FUSB302_STATUS1A_TOGSS2, "TOGSS2"},
    {FUSB302_STATUS1A_TOGSS1, "TOGSS1"},
    {FUSB302_STATUS1A_RXSOP2DB, "RXSOP2DB"},
    {FUSB302_STATUS1A_RXSOP1DB, "RXSOP1DB"},
    {FUSB302_STATUS1A_RXSOP, "RXSOP"},
};

/*------------ Interrupt Registers ------------*/
// Note: reading interrupts clears it
static const struct bit_name fusb302_interrupt_bits[] = {
    {FUSB302_INT_VBUSOK, "I_VBUSOK"},
    {FUSB302_INT_ACTIVITY, "I_ACTIVITY"},
    {FUSB302_INT_COMP_CHNG, "I_COMP_CHNG"},
    {FUSB302_INT_CRC_CHK, "I_CRC_CHK"},
    {FUSB302_INT_ALERT, "I_ALERT"},
    {FUSB302_INT_WAKE, "I_WAKE"},
    {FUSB302_INT_COLLISION, "I_COLLISION"},
    {FUSB302_INT_BC_LVL, "I_BC_LVL"},
};

static const struct bit_name fusb302_interrupta_bits[] = {
    {FUSB302_INTA_OCP_TEMP, "I_OCP_TEMP"},
    {FUSB302_INTA_TOGDONE, "I_TOGDONE"},
    {FUSB302_INTA_SOFTFAIL, "I_SOFTFAIL"},
    {FUSB302_INTA_RETRYFAIL, "I_RETRYFAIL"},
    {FUSB302_INTA_HARDSENT, "I_HARDSENT"},
    {FUSB302_INTA_TXSENT, "I_TXSENT"},
    {FUSB302_INTA_SOFTRST, "I_SOFTRST"},
    {FUSB302_INTA_HARDRST, "I_HARDRST"},
};

/*------------ Switches Registers ------------*/

static const struct bit_name fusb302_switches0_bits[] = {
    {FUSB302_SW0_PU_EN2, "PU_EN2"},
    {FUSB302_SW0_PU_EN1, "PU_EN1"},
    {FUSB302_SW0_VCONN_CC2, "VCONN_CC2"},
    {FUSB302_SW0_VCONN_CC1, "VCONN_CC1"},
    {FUSB302_SW0_MEAS_CC2, "MEAS_CC2"},
    {FUSB302_SW0_MEAS_CC1, "MEAS_CC1"},
    {FUSB302_SW0_PDWN2, "PDWN2"},
    {FUSB302_SW0_PDWN1, "PDWN1"},
};

static const struct bit_name fusb302_switches1_bits[] = {
    {FUSB302_SW1_POWERROLE, "POWERROLE"},
    {FUSB302_SW1_SPECREV1, "SPECREV1"},
    {FUSB302_SW1_SPECREV0, "SPECREV0"},
    {FUSB302_SW1_DATAROLE, "DATAROLE"},
    {BIT(3), NULL},
    {FUSB302_SW1_AUTO_GCRC, "AUTO_GCRC"},
    {FUSB302_SW1_TXCC2, "TXCC2"},
    {FUSB302_SW1_TXCC1, "TXCC1"},
};

/*------------ Control Registers ------------*/

static const struct bit_name fusb302_control0_bits[] = {
    {BIT(7), NULL},
    {FUSB302_CTL0_TX_FLUSH, "TX_FLUSH"},
    {FUSB302_CTL0_INT_MASK, "INT_MASK"},
    {BIT(4), NULL},
    {FUSB302_CTL0_HOST_CUR1, "HOST_CUR1"},
    {FUSB302_CTL0_HOST_CUR0, "HOST_CUR0"},
    {FUSB302_CTL0_AUTO_PRE, "AUTO_PRE"},
    {FUSB302_CTL0_TX_START, "TX_START"},
};

static const struct bit_name fusb302_control1_bits[] = {
    {BIT(7), NULL},
    {FUSB302_CTL1_ENSOP2DB, "ENSOP2DB"},
    {FUSB302_CTL1_ENSOP1DB, "ENSOP1DB"},
    {FUSB302_CTL1_BIST_MODE2, "BIST_MODE2"},
    {BIT(3), NULL},
    {FUSB302_CTL1_RX_FLUSH, "RX_FLUSH"},
    {FUSB302_CTL1_ENSOP2, "ENSOP2"},
    {FUSB302_CTL1_ENSOP1, "ENSOP1"},
};

static const struct bit_name fusb302_control2_bits[] = {
    {FUSB302_CTL2_TOG_SAVE_PWR2, "TOG_SAVE_PWR2"},
    {FUSB302_CTL2_TOG_SAVE_PWR1, "TOG_SAVE_PWR1"},
    {FUSB302_CTL2_TOG_RD_ONLY, "TOG_RD_ONLY"},
    {BIT(4), NULL},
    {FUSB302_CTL2_WAKE_EN, "WAKE_EN"},
    {FUSB302_CTL2_MODE_1, "MODE_1"},
    {FUSB302_CTL2_MODE_0, "MODE_0"},
    {FUSB302_CTL2_TOGGLE, "TOGGLE"},
};

static const struct bit_name fusb302_control3_bits[] = {
    {BIT(7), NULL},
    {FUSB302_CTL3_SEND_HARDRESET, "SEND_HARDRESET"},
    {FUSB302_CTL3_BIST_TMODE, "BIST_TMODE"},
    {FUSB302_CTL3_AUTO_HARDRESET, "AUTO_HARDRESET"},
    {FUSB302_CTL3_AUTO_SOFTRESET, "AUTO_SOFTRESET"},
    {FUSB302_CTL3_NRETRIES1, "NRETRIES1"},
    {FUSB302_CTL3_NRETRIES0, "NRETRIES0"},
    {FUSB302_CTL3_AUTO_RETRY, "AUTO_RETRY"},
};

/*------------ Mask Registers ------------*/

static const struct bit_name fusb302_mask_bits[] = {
    {FUSB302_MASK_VBUSOK, "VBUSOK"},
    {FUSB302_MASK_ACTIVITY, "ACTIVITY"},
    {FUSB302_MASK_COMP_CHNG, "COMP_CHNG"},
    {FUSB302_MASK_CRC_CHK, "CRC_CHK"},
    {FUSB302_MASK_ALERT, "ALERT"},
    {FUSB302_MASK_WAKE, "WAKE"},
    {FUSB302_MASK_COLLISION, "COLLISION"},
    {FUSB302_MASK_BC_LVL, "BC_LVL"},
};

static const struct bit_name fusb302_maska_bits[] = {
    {FUSB302_MASKA_OCP_TEMP, "OCP_TEMP"},
    {FUSB302_MASKA_TOGDONE, "TOGDONE"},
    {FUSB302_MASKA_SOFTFAIL, "SOFTFAIL"},
    {FUSB302_MASKA_RETRYFAIL, "RETRYFAIL"},
    {FUSB302_MASKA_HARDSENT, "HARDSENT"},
    {FUSB302_MASKA_TXSENT, "TXSENT"},
    {FUSB302_MASKA_SOFTRST, "SOFTRST"},
    {FUSB302_MASKA_HARDRST, "HARDRST"},
};

static const struct bit_name fusb302_maskb_bits[] = {
    {BIT(7), NULL},
    {BIT(6), NULL},
    {BIT(5), NULL},
    {BIT(4), NULL},
    {BIT(3), NULL},
    {BIT(2), NULL},
    {BIT(1), NULL},
    {FUSB302_MASKB_GCRCSENT, "GCRCSENT"},
};

#endif /* FUSB302_H */