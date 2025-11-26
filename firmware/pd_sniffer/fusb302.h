#ifndef FUSB302_H
#define FUSB302_H

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
#define FUSB302_SW1_AUTO_CRC        (1 << 2)
#define FUSB302_SW1_TXCC2           (1 << 1)
#define FUSB302_SW1_TXCC1           (1 << 0)

/* -----------------------------------------------------------
 * MEASURE (0x04)
 * ----------------------------------------------------------- */
#define FUSB302_MEAS_VBUS           (1 << 6)
#define FUSB302_MEAS_MDAC_POS       0
#define FUSB302_MEAS_MDAC_MASK      (0x3F << FUSB302_MEAS_MDAC_POS)

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
#define FUSB302_CTL2_TOGGLE         (1 << 0)

#define FUSB302_CTL2_MODE_UFP       (0 << FUSB302_CTL2_MODE_POS)
#define FUSB302_CTL2_MODE_DFP       (1 << FUSB302_CTL2_MODE_POS)
#define FUSB302_CTL2_MODE_DRP       (2 << FUSB302_CTL2_MODE_POS)

/* -----------------------------------------------------------
 * CONTROL3 (0x09)  — CORRECTED: hard reset = bit 6
 * ----------------------------------------------------------- */
#define FUSB302_CTL3_SEND_HARD_RESET (1 << 6)
#define FUSB302_CTL3_BIST_TMODE     (1 << 5)
#define FUSB302_CTL3_AUTO_HARDRESET (1 << 4)
#define FUSB302_CTL3_AUTO_SOFTRESET (1 << 3)
#define FUSB302_CTL3_NRETRIES_POS   1
#define FUSB302_CTL3_NRETRIES_MASK  (3 << FUSB302_CTL3_NRETRIES_POS)
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

/* -----------------------------------------------------------
 * FIFO (0x43)
 * ----------------------------------------------------------- */
#define FUSB302_FIFO_BYTE           0x43

#endif /* FUSB302_H */