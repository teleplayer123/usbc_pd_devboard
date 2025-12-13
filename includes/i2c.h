#include <stdint.h>
#include <stdlib.h>

int i2c_xfer(uint32_t i2c, uint8_t addr, uint8_t *out, int out_len, uint8_t *in, int in_len);
uint8_t i2c_read_reg(uint32_t i2c, uint8_t reg);
void i2c_write_reg(uint32_t i2c, uint8_t reg, uint8_t val);
void i2c_read_fifo(uint32_t i2c, uint8_t *data, size_t len);