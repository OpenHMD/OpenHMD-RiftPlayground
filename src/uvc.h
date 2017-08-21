#ifndef __UVC_H__
#define __UVC_H__

int uvc_set_cur(libusb_device_handle *dev, uint8_t entity, uint8_t selector,
		unsigned char *data, uint16_t wLength);

int uvc_get_cur(libusb_device_handle *dev, uint8_t entity, uint8_t selector,
		unsigned char *data, uint16_t wLength);

#endif /* __UVC_H__ */
