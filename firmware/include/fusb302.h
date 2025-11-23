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


#ifdef __cplusplus
}
#endif

#endif /* FUSB302_H */