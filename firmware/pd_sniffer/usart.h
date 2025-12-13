#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "fusb302.h"

char usart_getc(void);
void usart_send_char(char c);
void usart_printf(const char *format, ...);
void print_byte_as_bits(uint8_t byte, uint8_t reg);
void dump_bits(uint8_t reg, const struct bit_name *tbl);
void hexdump(const uint8_t *data, size_t len);
bool uart_rx_ready(void);