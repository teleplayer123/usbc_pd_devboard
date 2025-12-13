#include <libopencm3/stm32/i2c.h>
#include "fusb302.h"
#include "i2c.h"

/*---- I2C functions ----*/
/*
void i2c_transfer7(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn);
    i2c: The base address of the I2C peripheral
    addr: The 7-bit I2C slave address
    w: A pointer to the data buffer to be written to the slave
    wn: The number of bytes to write
    r: A pointer to the buffer where the data read from the slave will be stored
    rn: The number of bytes to read from the slave
*/

int i2c_xfer(uint32_t i2c, uint8_t addr, uint8_t *out, int out_len, uint8_t *in, int in_len) {
    if (out_len > 0 && in_len > 0) {
        // Write then read
        i2c_transfer7(i2c, addr, out, out_len, in, in_len);
    } else if (out_len > 0) {
        // Write only
        i2c_transfer7(i2c, addr, out, out_len, NULL, 0);
    } else if (in_len > 0) {
        // Read only
        i2c_transfer7(i2c, addr, NULL, 0, in, in_len);
    }
    return 0;
}

uint8_t i2c_read_reg(uint32_t i2c, uint8_t reg) {
    uint8_t val;
    i2c_transfer7(i2c, FUSB302_ADDR, &reg, 1, &val, 1);
    return val;
}

void i2c_write_reg(uint32_t i2c, uint8_t reg, uint8_t val) {
    uint8_t tx_buf[2] = {reg, val};
    i2c_transfer7(i2c, FUSB302_ADDR, tx_buf, 2, NULL, 0);
}

void i2c_read_fifo(uint32_t i2c, uint8_t *data, size_t len) {
    uint8_t reg = FUSB302_REG_FIFOS;
    i2c_transfer7(i2c, FUSB302_ADDR, &reg, 1, data, len);
}