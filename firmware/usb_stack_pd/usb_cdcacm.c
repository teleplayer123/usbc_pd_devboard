#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <stdio.h>
#include "usb_cdcacm.h"

static usbd_device *cdcacm_dev;
static void (*cdcacm_rx_cb_user)(uint8_t*, int) = 0;

static uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1209,      /* example PID/VID */
    .idProduct = 0x0001,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/* CDC endpoints/configuration */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct usb_cdc_header_descriptor cdc_header = {
    .bFunctionLength = sizeof(cdc_header),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
    .bcdCDC = 0x0110,
};

static const struct usb_cdc_call_management_descriptor cdc_call_mgmt = {
    .bFunctionLength = sizeof(cdc_call_mgmt),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
    .bmCapabilities = 0,
    .bDataInterface = 1,
};

static const struct usb_cdc_acm_descriptor cdc_acm = {
    .bFunctionLength = sizeof(cdc_acm),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_ACM,
    .bmCapabilities = 0,
};

static const struct usb_cdc_union_descriptor cdc_union = {
    .bFunctionLength = sizeof(cdc_union),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_UNION,
    .bControlInterface = 0,
    .bSubordinateInterface0 = 1,
};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} cdc_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(cdc_header),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength = sizeof(cdc_call_mgmt),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(cdc_acm),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(cdc_union),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    },
};

// static const struct {
//     struct usb_cdc_header_descriptor header;
//     struct usb_cdc_call_management_descriptor call_mgmt;
//     struct usb_cdc_acm_descriptor acm;
//     struct usb_cdc_union_descriptor cdc_union;
// } __attribute__((packed)) cdc_functional_descriptors = {
//     .header = { .bFunctionLength = sizeof(struct usb_cdc_header_descriptor), .bDescriptorType = CS_INTERFACE, .bDescriptorSubtype = USB_CDC_TYPE_HEADER, .bcdCDC = 0x0110 },
//     .call_mgmt = { .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor), .bDescriptorType = CS_INTERFACE, .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT, .bmCapabilities = 0, .bDataInterface = 1 },
//     .acm = { .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor), .bDescriptorType = CS_INTERFACE, .bDescriptorSubtype = USB_CDC_TYPE_ACM, .bmCapabilities = 2 },
//     .cdc_union = { .bFunctionLength = sizeof(struct usb_cdc_union_descriptor), .bDescriptorType = CS_INTERFACE, .bDescriptorSubtype = USB_CDC_TYPE_UNION, .bControlInterface = 0, .bSubordinateInterface0 = 1 }
// };

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,
    .extra = &cdc_functional_descriptors,
    .extralen = sizeof(cdc_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {
    { .num_altsetting = 1, .altsetting = comm_iface, },
    { .num_altsetting = 1, .altsetting = data_iface, },
};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,
    .interface = ifaces,
};

static const char *usb_strings[] = {
    "PD Debugger",
    "USB-I2C Bridge",
    "123456",
};

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    uint8_t buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));
    if (len && cdcacm_rx_cb_user) cdcacm_rx_cb_user(buf, len);
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;
    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
}

usbd_device *usb_cdcacm_init(void (*rx_cb)(uint8_t *buf, int len))
{
    cdcacm_rx_cb_user = rx_cb;
    cdcacm_dev = usbd_init(&st_usbfs_v2_usb_driver, &dev_descr, &config,
                           usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(cdcacm_dev, cdcacm_set_config);
    return cdcacm_dev;
}

void usb_cdcacm_write(uint8_t *buf, int len)
{
    usbd_ep_write_packet(cdcacm_dev, 0x82, buf, len);
}

/* Updated Printf function for USB */
int usb_printf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    /* Send via Bulk In endpoint 0x82 */
    int sent = 0;
    while (sent < len) {
        int chunk = (len - sent > 64) ? 64 : (len - sent);
        while (usbd_ep_write_packet(cdcacm_dev, 0x82, &buf[sent], chunk) == 0);
        sent += chunk;
    }
    return len;
}
