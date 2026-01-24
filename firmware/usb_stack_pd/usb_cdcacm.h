#include <libopencm3/usb/usbd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>

usbd_device *usb_cdcacm_init(void (*rx_cb)(uint8_t *buf, int len));
void usb_cdcacm_write(uint8_t *buf, int len);
int usb_printf(const char *format, ...);
