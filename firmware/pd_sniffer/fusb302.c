/*
 * fusb302.c
 *
 * FUSB302B PHY sniffer/debug helper for libopencm3.
 * - Reads RX FIFO tokens and constructs PD packets (header + payload).
 * - Designed for analysis/debugging (no PD policy/state machine).
 *
 * Requires:
 *   - libopencm3 (i2c_transfer7, optional i2c init helper uses rcc/gpio)
 *   - the FUSB302 register header with FUSB302_ prefixed macros:
 *       #include "fusb302_regs.h"
 *
 * Datasheet reference (uploaded): /mnt/data/FUSB302B-D.PDF
 *
 * Public API:
 *   int  fusb302_i2c_init(void);        // optional helper for libopencm3 I2C1
 *   int  fusb302_init_debug(void);      // recommended initial configuration
 *   int  fusb302_rx_available(void);    // returns 1 if FIFO has data
 *   int  fusb302_read_packet(fusb302_packet_t *pkt); // blocking read
 *   void fusb302_dump_packet(const fusb302_packet_t *p);
 *
 * Notes:
 *  - This file contains a complete libopencm3 I2C backend using i2c_transfer7().
 *  - If you prefer to provide your own I2C code, replace fusb302_i2c_write/read() below.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "fusb302_regs.h"

/* ---------- libopencm3 includes (used by I2C helper) ---------- */
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/* Choose I2C peripheral here (change if using I2C2/I2C3) */
#ifndef FUSB302_I2C_PERIPH
#define FUSB302_I2C_PERIPH I2C1
#endif

/* FUSB302 7-bit I2C address is defined in header as FUSB302_I2C_ADDR */

/* ---------- Low-level I2C helpers (libopencm3) ---------- */

/* Write to a register (send register address followed by data)
 * Returns 0 on success, negative on error.
 */
static int fusb302_i2c_write(uint8_t reg, const uint8_t *buf, size_t len)
{
    /* Build transfer buffer: reg + data */
    uint8_t txbuf[1 + 64]; /* enough for typical writes */
    if (len > 64) return -1;
    txbuf[0] = reg;
    if (len)
        memcpy(&txbuf[1], buf, len);

    int rc = i2c_transfer7(FUSB302_I2C_PERIPH, FUSB302_I2C_ADDR, txbuf, (uint32_t)(len + 1), NULL, 0);
    return (rc == 0) ? 0 : -2;
}

/* Read N bytes starting at register 'reg' (writes reg then reads len bytes) */
static int fusb302_i2c_read(uint8_t reg, uint8_t *buf, size_t len)
{
    int rc = i2c_transfer7(FUSB302_I2C_PERIPH, FUSB302_I2C_ADDR, &reg, 1, buf, (uint32_t)len);
    return (rc == 0) ? 0 : -2;
}

/* ---------- register read/write ---------- */

static int fusb302_write_reg(uint8_t reg, uint8_t val)
{
    return fusb302_i2c_write(reg, &val, 1);
}

static int fusb302_read_reg(uint8_t reg, uint8_t *val)
{
    return fusb302_i2c_read(reg, val, 1);
}

/* Multi byte read/write */
static int fusb302_write_regs(uint8_t reg, const uint8_t *buf, size_t len)
{
    return fusb302_i2c_write(reg, buf, len);
}

static int fusb302_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return fusb302_i2c_read(reg, buf, len);
}

/* ---------- Packet structures and parser helpers ---------- */

#define FUSB302_MAX_PACKET_BYTES  64

typedef enum {
    FUSB302_SOP_INVALID = 0,
    FUSB302_SOP_SOP,
    FUSB302_SOP_SOP_PRIME,
    FUSB302_SOP_SOP_DPRIME
} fusb302_sop_t;

typedef struct {
    fusb302_sop_t sop;
    uint8_t header[2];                       /* PD Header (2 bytes) */
    uint8_t payload[FUSB302_MAX_PACKET_BYTES];
    size_t payload_len;
} fusb302_packet_t;

/* Helper to read a single byte from FIFO register */
static int fusb302_fifo_read_byte(uint8_t *b)
{
    return fusb302_read_reg(FUSB302_REG_FIFOS, b);
}

/* Return 1 if RX FIFO has data (STATUS1 RX_EMPTY == 0), 0 otherwise.
 * Returns 0 on I2C error as a conservative default.
 */
int fusb302_rx_available(void)
{
    uint8_t status1;
    if (fusb302_read_reg(FUSB302_REG_STATUS1, &status1))
        return 0;
    return ((status1 & FUSB302_STATUS1_RX_EMPTY) == 0) ? 1 : 0;
}

/* Read one PD packet from FIFO and fill pkt.
 * This is a polling/blocking routine: it waits until FIFO has data and completes parsing.
 *
 * Returns:
 *   0  : success (pkt filled)
 *  -1  : bad parameter
 *  -2  : i2c error reading status
 *  -3  : i2c error reading fifo
 */
int fusb302_read_packet(fusb302_packet_t *pkt)
{
    if (!pkt) return -1;
    memset(pkt, 0, sizeof(*pkt));

    /* Wait until RX FIFO not empty */
    uint8_t status1;
    for (;;) {
        if (fusb302_read_reg(FUSB302_REG_STATUS1, &status1)) return -2;
        if ((status1 & FUSB302_STATUS1_RX_EMPTY) == 0) break;
        /* Nothing available: you may want to sleep/yield in your application */
    }

    /* Now parse tokens. Approach:
     *  - Read token bytes from FIFO until we assemble a packet:
     *    * Detect SOP token (SOP / SOP' / SOP'')
     *    * Following tokens will include PACKSYM tokens giving counts of subsequent data bytes
     *    * PD header is two bytes (first two data bytes after PACKSYM)
     *    * Remaining data bytes are payload (N bytes) — payload_len tracked
     *  - End when we detect EOP (0x14) or when FIFO empty after reading meaningful data.
     *
     * Note: Datasheet token mapping: PACKSYM tokens are 0x80..0x9F (0b100xxxxx)
     * SOP tokens: 0xE0..0xFF (SOP), 0xC0..0xDF (SOP'), 0xA0..0xBF (SOP'')
     */

    int in_packet = 0;
    int seen_header = 0;
    size_t header_received = 0;
    pkt->payload_len = 0;

    while (1) {
        uint8_t tok;
        if (fusb302_fifo_read_byte(&tok)) return -3;

        /* PACKSYM: 0b100xxxxx (0x80..0x9F) */
        if ((tok & 0xE0) == 0x80) {
            uint8_t count = tok & 0x1F;
            for (uint8_t i = 0; i < count; ++i) {
                uint8_t d;
                if (fusb302_fifo_read_byte(&d)) return -3;

                if (in_packet) {
                    if (!seen_header) {
                        /* Fill header (two bytes) first */
                        if (header_received == 0) {
                            pkt->header[0] = d;
                            header_received = 1;
                        } else if (header_received == 1) {
                            pkt->header[1] = d;
                            header_received = 2;
                            seen_header = 1;
                        } else {
                            printf("Unexpected header byte received: %02X\n", d);
                        }
                    } else {
                        if (pkt->payload_len < sizeof(pkt->payload)) {
                            pkt->payload[pkt->payload_len++] = d;
                        }
                    }
                } else {
                    printf("Data byte received outside of packet: %02X\n", d);
                }
            }
            /* after reading data, loop for next token */
            continue;
        }

        /* SOP tokens detection: top-3 bits (111,110,101) */
        uint8_t top3 = (tok & 0xE0);
        if (top3 == 0xE0 || top3 == 0xC0 || top3 == 0xA0) {
            in_packet = 1;
            seen_header = 0;
            header_received = 0;
            pkt->payload_len = 0;

            if (top3 == 0xE0) pkt->sop = FUSB302_SOP_SOP;
            else if (top3 == 0xC0) pkt->sop = FUSB302_SOP_SOP_PRIME;
            else pkt->sop = FUSB302_SOP_SOP_DPRIME;

            /* continue to next token (PACKSYM expected) */
            continue;
        }

        /* EOP token (0x14) signals end of packet (per datasheet tokens table) */
        if (tok == 0x14) {
            if (in_packet) {
                /* If we have at least header bytes, consider packet complete */
                if (header_received >= 2 || pkt->payload_len > 0) {
                    return 0;
                }
            }
            continue;
        }

        /* Other tokens - if we encounter new SOP when building packet, we can treat current as complete */
        if (in_packet && ((tok & 0xE0) == 0xE0 || (tok & 0xE0) == 0xC0 || (tok & 0xE0) == 0xA0)) {
            /* new SOP encountered -> previous packet was ended implicitly */
            return 0;
        }

        /* Check if FIFO is empty: if so and we've started packet -> finish */
        if (fusb302_read_reg(FUSB302_REG_STATUS1, &status1)) return -2;
        if ((status1 & FUSB302_STATUS1_RX_EMPTY) && in_packet && (header_received >= 1 || pkt->payload_len > 0)) {
            /* We read all available bytes for the packet */
            return 0;
        }

        /* Otherwise continue reading tokens */
    }

    /* unreachable */
    // return 0;
}

/* Dump packet to stdout (printf) for debugging */
void fusb302_dump_packet(const fusb302_packet_t *p)
{
    if (!p) return;
    const char *sop = "INVALID";
    if (p->sop == FUSB302_SOP_SOP) sop = "SOP";
    else if (p->sop == FUSB302_SOP_SOP_PRIME) sop = "SOP'";
    else if (p->sop == FUSB302_SOP_SOP_DPRIME) sop = "SOP''";

    printf("FUSB302 PD Packet (%s):\n", sop);
    printf("  Header: 0x%02X 0x%02X\n", p->header[0], p->header[1]);
    printf("  Payload (%zu bytes):", p->payload_len);
    for (size_t i = 0; i < p->payload_len; ++i) {
        if ((i % 16) == 0) printf("\n    ");
        printf("%02X ", p->payload[i]);
    }
    printf("\n");
}

/* Minimal debug init that configures the chip for sniffing:
 *  - Turn on power bits (bandgap, receiver, measure, oscillator)
 *  - Clear masks so interrupts may fire (you can adjust)
 *  - Enable reception of SOP, SOP', SOP'' in CONTROL1 ENSOP bits
 *  - Optionally enable AUTO_CRC in SWITCHES1 if you want the FUSB302 to ACK automatically
 */
int fusb302_init_debug(void)
{
    int r;
    uint8_t v;

    /* Power up: PWR = 0x07 (PWR[3:0] = 0111) as recommended */
    v = 0x07;
    r = fusb302_write_reg(FUSB302_REG_POWER, v);
    if (r) return r;

    /* Unmask interrupts (allow them) - set mask registers to 0 */
    v = 0x00;
    r = fusb302_write_reg(FUSB302_REG_MASK, v);
    if (r) return r;
    r = fusb302_write_reg(FUSB302_REG_MASKA, 0x00);
    if (r) return r;
    r = fusb302_write_reg(FUSB302_REG_MASKB, 0x00);
    if (r) return r;

    /* Enable receiving SOP / SOP' / SOP'' (CONTROL1 ENSOP bits)
     * Use CTL1 bits from header:
     *   FUSB302_CTL1_ENSOP1, FUSB302_CTL1_ENSOP2,
     *   FUSB302_CTL1_ENSOP1DB, FUSB302_CTL1_ENSOP2DB
     */
#ifdef FUSB302_CTL1_ENSOP1DB
    v = FUSB302_CTL1_ENSOP1 | FUSB302_CTL1_ENSOP2 | FUSB302_CTL1_ENSOP1DB | FUSB302_CTL1_ENSOP2DB;
#else
    /* fallback if header naming differs */
    r = fusb302_read_reg(FUSB302_REG_CONTROL1, &v);
    if (r) return r;
    v |= 0x63; /* enable SOP, SOP', SOP'' debug bits if layout differs; adjust if needed */
#endif
    r = fusb302_write_reg(FUSB302_REG_CONTROL1, v);
    if (r) return r;

    /* Clear interrupts/status by reading them */
    uint8_t tmp;
    fusb302_read_reg(FUSB302_REG_INTERRUPTA, &tmp);
    fusb302_read_reg(FUSB302_REG_INTERRUPTB, &tmp);
    fusb302_read_reg(FUSB302_REG_INTERRUPT, &tmp);

    /* Optionally: enable AUTO_CRC in SWITCHES1 to let chip auto-GoodCRC (disable if you want raw log)
     * Uncomment the next block to enable AUTO_CRC:
     */
#if 0
    r = fusb302_read_reg(FUSB302_REG_SWITCHES1, &v);
    if (r) return r;
    v |= FUSB302_SW1_AUTO_CRC;
    r = fusb302_write_reg(FUSB302_REG_SWITCHES1, v);
    if (r) return r;
#endif

    return 0;
}

/* Main polling loop */
void fusb302_poll_and_dump(void)
{
    fusb302_packet_t pkt;
    while (1) {
        if (fusb302_rx_available()) {
            int rc = fusb302_read_packet(&pkt);
            if (rc == 0) {
                fusb302_dump_packet(&pkt);
            } else {
                printf("Error: fusb302_read_packet rc=%d\n", rc);
            }
        }
        /* Replace busy-looping with a small delay or yield in your system */
    }
}
