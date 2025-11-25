/*
 * fusb302.h
 *
 * Definitions and simple helpers for FUSB302 USB Type-C controller
 */

#ifndef FUSB302_H
#define FUSB302_H

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int   uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef signed int     int32_t;
typedef signed short   int16_t;
typedef signed char    int8_t;

#define FUSB302_ADDR                0x22   // 7-bit address

#define FUSB302_REG_DEVICE_ID       0x01
#define FUSB302_REG_SWITCHES0       0x02
#define FUSB302_REG_SWITCHES1       0x03
#define FUSB302_REG_MEASURE         0x04
#define FUSB302_REG_SLICE           0x05
#define FUSB302_REG_CONTROL0        0x06
#define FUSB302_REG_CONTROL1        0x07
#define FUSB302_REG_CONTROL2        0x08
#define FUSB302_REG_CONTROL3        0x09
#define FUSB302_REG_MASK1           0x0A
#define FUSB302_REG_POWER           0x0B
#define FUSB302_REG_RESET           0x0C
#define FUSB302_REG_OCP             0x0D
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

/* SWITCHES0 Bits */
#define SW0_CC2_PU_EN        (1 << 7)   // 1: apply host pull-up current to CC2 pin
#define SW0_CC1_PU_EN        (1 << 6)   // 1: apply host pull-up current to CC1 pin
#define SW0_VCONN_CC2_EN     (1 << 5)   // 1: turn on VCONN current to CC2 pin
#define SW0_VCONN_CC1_EN     (1 << 4)   // 1: turn on VCONN current to CC1 pin
#define SW0_MEAS_CC2         (1 << 3)   // 1: measure voltage on CC2 pin
#define SW0_MEAS_CC1         (1 << 2)   // 1: measure voltage on CC1 pin
#define SW0_PDWN2            (1 << 1)   // 1: device pull down on CC2. 0: no pull down
#define SW0_PDWN1            (1 << 0)   // 1: device pull down on CC1. 0: no pull down

/* SWITCHES1 Bits */
#define SW1_POWERROLE       (1 << 7)
#define SW1_SPECREV1        (1 << 6)
#define SW1_SPECREV0        (1 << 5)
#define SW1_DATAROLE        (1 << 4)
#define SW1_AUTO_CRC        (1 << 2)
#define SW1_TXCC2           (1 << 1)
#define SW1_TXCC1           (1 << 0)

/* CONTROL0 Bits */
#define CTL0_TX_FLUSH       (1 << 6)
#define CTL0_INT_MASK       (1 << 5)
#define CTL0_HOST_CUR_POS   2
#define CTL0_HOST_CUR_MASK  (3 << CTL0_HOST_CUR_POS)
#define CTL0_AUTO_PRE       (1 << 1)
#define CTL0_TX_START       (1 << 0)

/* CONTROL1 Bits */
#define FUSB_CTL1_ENSOP2DB      (1 << 6)
#define FUSB_CTL1_ENSOP1DB      (1 << 5)
#define FUSB_CTL1_BIST_MODE2    (1 << 4)
#define FUSB_CTL1_RX_FLUSH      (1 << 2)
#define FUSB_CTL1_ENSOP2        (1 << 1)
#define FUSB_CTL1_ENSOP1        (1 << 0)    

/* CONTROL2 Bits */
#define CTL2_TOG_SAVE_PWR2 (1 << 7)
#define CTL2_TOG_SAVE_PWR1 (1 << 6)
#define CTL2_TOG_RD_ONLY   (1 << 5)
#define CTL2_WAKE_EN       (1 << 3)
#define CTL2_MODE_POS      1
#define CTL2_MODE_MASK     (3 << CTL2_MODE_POS)
#define CTL2_TOGGLE        (1 << 0)


#ifdef __cplusplus
}
#endif

#endif /* FUSB302_H */