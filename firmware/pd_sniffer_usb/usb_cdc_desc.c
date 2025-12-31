#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Strings                                                                     */
/* -------------------------------------------------------------------------- */

const char *usb_strings[] = {
    "PD Debugger",
    "USB-C PD Sniffer",
    "0001",
};

/* -------------------------------------------------------------------------- */
/* Endpoints                                                                   */
/* -------------------------------------------------------------------------- */

static const struct usb_endpoint_descriptor comm_ep = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83, /* IN */
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
};

static const struct usb_endpoint_descriptor data_ep[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01, /* OUT */
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82, /* IN */
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

/* -------------------------------------------------------------------------- */
/* Interfaces                                                                  */
/* -------------------------------------------------------------------------- */

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdc_func_desc = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    }
};

static const struct usb_interface_descriptor comm_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .endpoint = &comm_ep,
    .extra = &cdc_func_desc,
    .extralen = sizeof(cdc_func_desc),
};

static const struct usb_interface_descriptor data_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .endpoint = data_ep,
};

static const struct usb_interface interfaces[] = {{
    .num_altsetting = 1,
    .altsetting = &comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = &data_iface,
}};

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

const struct usb_config_descriptor config_descr = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength =
        USB_DT_CONFIGURATION_SIZE +
        USB_DT_INTERFACE_SIZE * 2 +
        USB_DT_ENDPOINT_SIZE * 3 +
        sizeof(cdc_func_desc),
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .bmAttributes = 0x80,
    .bMaxPower = 50,
    .interface = interfaces,
};

const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,            /* USB 2.0 */
    .bDeviceClass = USB_CLASS_CDC,      /* CDC at device level is OK */
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,       /* EP0 max packet size */
    .idVendor = 0x1209,          /* pid.codes shared VID (recommended) */
    .idProduct = 0x0001,         /* choose any free PID */
    .bcdDevice = 0x0100,         /* device release */
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/* -------------------------------------------------------------------------- */
/* CDC control requests (REQUIRED FOR WINDOWS)                                 */
/* -------------------------------------------------------------------------- */

static enum usbd_request_return_codes cdc_control_request(
    usbd_device *dev,
    struct usb_setup_data *req,
    uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *, struct usb_setup_data *)
) {
    (void)dev;
    (void)complete;

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
        return USBD_REQ_HANDLED;

    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding))
            return USBD_REQ_NOTSUPP;
        return USBD_REQ_HANDLED;

    case USB_CDC_REQ_GET_LINE_CODING:
        return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NOTSUPP;
}

/* -------------------------------------------------------------------------- */
/* Public hook                                                                 */
/* -------------------------------------------------------------------------- */

void cdcacm_set_config(usbd_device *dev)
{
    usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
        dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        cdc_control_request);
}
