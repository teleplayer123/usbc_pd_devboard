#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fusb302.h"


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
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

static void i2c_setup(void) {
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);

    /* Hardware reset via RCC */
    rcc_peripheral_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);
    rcc_peripheral_clear_reset(&RCC_APB1RSTR, RCC_APB1RSTR_I2C1RST);

    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

/* simple blocking getchar/putchar */
int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i=0; i<len; i++) usart_send_blocking(USART2, ptr[i]);
    return len;
}

static char usart_getc(void) { 
    return usart_recv_blocking(USART2); 
}

/* I2C helpers */
/*
void i2c_transfer7(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn);
    i2c: The base address of the I2C peripheral
    addr: The 7-bit I2C slave address
    w: A pointer to the data buffer to be written to the slave
    wn: The number of bytes to write
    r: A pointer to the buffer where the data read from the slave will be stored
    rn: The number of bytes to read from the slave
*/

static int fusb_read_reg(uint32_t i2c, uint8_t reg, uint8_t *val) {
    i2c_transfer7(i2c, FUSB302_ADDR, &reg, 1, val, 1);
    return 0;
}
static int fusb_write_reg(uint32_t i2c, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_transfer7(i2c, FUSB302_ADDR, buf, 2, NULL, 0);
    return 0;
}

static int fusb_read_reg_nbytes(uint32_t i2c, uint8_t reg, uint8_t *buf, size_t nbytes) {
    i2c_transfer7(i2c, FUSB302_ADDR, &reg, 1, buf, nbytes);
    return 0;
}

static void fusb_write_reg_nbytes(uint32_t i2c, uint8_t reg, const uint8_t *buf, size_t nbytes) {
    uint8_t wbuf_size = 1 + nbytes;
    uint8_t *wbuf = malloc(wbuf_size);
    wbuf[0] = reg;
    memcpy(&wbuf[1], buf, nbytes);
    i2c_transfer7(i2c, FUSB302_ADDR, wbuf, 1 + nbytes, NULL, 0);
}

/* low level i2c scan */
static bool i2c_probe_addr(uint32_t i2c, uint8_t addr) {

    /* clear flags */
    I2C_ICR(i2c) = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    /* send address */
    I2C_CR2(i2c) =
        (addr << 1) |      // address in bits 7:1
        (0 << 16)  |       // number of bytes
        I2C_CR2_START;     // generate START

    /* wait for either ACK or NACK */
    while (1) {
        uint32_t isr = I2C_ISR(i2c);

        if (isr & I2C_ISR_NACKF) {
            I2C_ICR(i2c) = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
            return false;  // NACK means no device
        }

        if (isr & I2C_ISR_STOPF) {
            I2C_ICR(i2c) = I2C_ICR_STOPCF;
            return true;   // STOP with no NACK means device responded
        }
    }
}

/* ---------- Packet structures and parser helpers ---------- */
static int fusb302_i2c_write(uint8_t reg, const uint8_t *buf, size_t len)
{
    /* Build transfer buffer: reg + data */
    uint8_t txbuf[1 + 64]; /* enough for typical writes */
    if (len > 64) return -1;
    txbuf[0] = reg;
    if (len)
        memcpy(&txbuf[1], buf, len);

    i2c_transfer7(I2C1, FUSB302_ADDR, txbuf, (uint32_t)(len + 1), NULL, 0);
    return 0;
}

/* Read N bytes starting at register 'reg' (writes reg then reads len bytes) */
static int fusb302_i2c_read(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_transfer7(I2C1, FUSB302_ADDR, &reg, 1, buf, (uint32_t)len);
    return 0;
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
    if (fusb302_read_reg(FUSB302_REG_STATUS1, &status1)) return 0;
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

    uint8_t status1;

    if (fusb302_read_reg(FUSB302_REG_STATUS1, &status1)) return -2;
    if ((status1 & FUSB302_STATUS1_RX_EMPTY) == 0) {
        printf("Data available in FIFO, starting read...\n");
    } else {
        /* no data */
        return -4;
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
        }

        /* EOP token (0x14) signals end of packet (per datasheet tokens table) */
        if (tok == 0x14) {
            if (in_packet) {
                /* If we have at least header bytes, consider packet complete */
                if (header_received >= 2 || pkt->payload_len > 0) {
                    return 0;
                }
            }
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

    }
    return 0;
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
void fusb302_poll_and_dump(uint8_t n_packets)
{
    fusb302_packet_t pkt;
    uint8_t count = 0;
    printf("Starting FUSB302 packet dump...\r\n");
    while (1) {
        if (count >= n_packets) {
            printf("Completed dumping %u packets.\r\n", count);
            break;
        }
        if (fusb302_rx_available()) {
            int rc = fusb302_read_packet(&pkt);
            if (rc == 0) {
                fusb302_dump_packet(&pkt);
                count++;
            } else {
                printf("Error: fusb302_read_packet rc=%d\n", rc);
                break;
            }
        }
    }
    printf("FUSB302 packet dump finished.\r\n");
}


/* CLI parser */
static void handle_command(char *line) {
    if (line[0] == 'r') {
        uint8_t reg = (uint8_t)strtol(&line[1], NULL, 0);
        uint8_t val;
        fusb_read_reg(I2C1, reg, &val);
        printf("read[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 'w') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        p = strtok(NULL, " ");
        if (!p) { printf("usage: w <reg> <val>\r\n"); return; }
        uint8_t val = (uint8_t)strtol(p, NULL, 0);
        fusb_write_reg(I2C1, reg, val);
        printf("write[0x%02X] = 0x%02X\r\n", reg, val);
    } else if (line[0] == 's') {
        printf("Scanning I2C...\r\n");
        for (uint8_t addr=1; addr<0x7F; addr++) {
            if (i2c_probe_addr(I2C1, addr)) {
                printf("Probed 0x%02X\r\n", addr);
            }
        }
    } else if (line[0] == 'b') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("bulk read usage: b <reg>\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        size_t nbytes = 80;
        uint8_t buf[80]; 
        fusb_read_reg_nbytes(I2C1, reg, buf, nbytes);
        printf("read[0x%02X] = ", reg);
        for (int i=0; i<80; i++) {
            printf("0x%02X ", buf[i]);
        }
        printf("\r\n");
    } else if (line[0] == 'n') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("bulk write usage: n <reg> <val1> <val2> ...\r\n"); return; }
        uint8_t reg = (uint8_t)strtol(p, NULL, 0);
        uint8_t buf[40];
        size_t nbytes = 0;
        while ((p = strtok(NULL, " ")) != NULL && nbytes < sizeof(buf)) {
            buf[nbytes++] = (uint8_t)strtol(p, NULL, 0);
        }
        fusb_write_reg_nbytes(I2C1, reg, buf, nbytes);
        printf("bulk wrote %u bytes to reg 0x%02X\r\n", (unsigned)nbytes, reg);
    } else if (line[0] == 'f') {
        char *p = strtok(&line[1], " ");
        if (!p) { printf("fusb302 packet dump: f <n_packets>\r\n"); return; }
        uint8_t n_packets = (uint8_t)strtol(p, NULL, 0);
        fusb302_init_debug();
        fusb302_poll_and_dump(n_packets);
    } else {
        printf("Commands:\r\n  r <reg>\r\n  w <reg> <val>\r\n  s (scan)\r\n  b <reg>\r\n  n <reg> <val1> <val2> ...\r\n  f <n_packets>\r\n");
    }
}

int main(void) {
    clock_setup();
    usart_setup();
    i2c_setup();
    printf("\r\nFUSB302 I2C test\r\n> ");

    char line[32]; int pos=0;
    while (1) {
        char c = usart_getc();
        if (c=='\r' || c=='\n') {
            line[pos]=0;
            printf("\r\n");
            handle_command(line);
            pos=0;
            printf("> ");
        } else if (pos < (int)sizeof(line)-1) {
            usart_send_blocking(USART2, c); /* echo */
            line[pos++]=c;
        }
    }
}
