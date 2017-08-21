#include <errno.h>
#include <libusb.h>
#include <stdint.h>
#include <stdio.h>

#define SET_CUR 0x01
#define GET_CUR 0x81
#define TIMEOUT 1000

#define CONTROL_IFACE   0

int uvc_set_cur(libusb_device_handle *dev, uint8_t entity, uint8_t selector,
		unsigned char *data, uint16_t wLength)
{
	uint8_t bmRequestType = LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = SET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | CONTROL_IFACE;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		fprintf(stderr, "failed to transfer SET CUR %u %u: %d %d %m\n",
			entity, selector, ret, errno);
	}
	return ret;
}

int uvc_get_cur(libusb_device_handle *dev, uint8_t entity, uint8_t selector,
		unsigned char *data, uint16_t wLength)
{
	uint8_t bmRequestType = 0x80 | LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = GET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | CONTROL_IFACE;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		fprintf(stderr, "failed to transfer GET CUR %u %u: %d %d %m\n",
			entity, selector, ret, errno);
	}
	return ret;
}
