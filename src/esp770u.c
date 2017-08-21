#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "uvc.h"

#define XU_ENTITY       4

#define REG_SEL         3

int esp770u_read_reg(libusb_device_handle *dev, uint8_t reg, uint8_t *val)
{
	unsigned char buf[4] = { 0x82, 0xf0, reg };
	int ret;

	ret = uvc_set_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, 0, XU_ENTITY, REG_SEL, buf, 3);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x82 || buf[2] != 0x00)
		fprintf(stderr, "read_reg(0x%x): %02x %02x %02x\n",
			reg, buf[0], buf[1], buf[2]);
	*val = buf[1];
	return ret;
}

int esp770u_read_reg_f1(libusb_device_handle *dev, uint8_t reg, uint8_t *val)
{
	unsigned char buf[4] = { 0x82, 0xf1, reg };
	int ret;

	ret = uvc_set_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, 0, XU_ENTITY, REG_SEL, buf, 3);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x82 || buf[2] != 0x00)
		fprintf(stderr, "read_reg(0x%x): %02x %02x %02x\n",
			reg, buf[0], buf[1], buf[2]);
	*val = buf[1];
	return ret;
}

int esp770u_write_reg(libusb_device_handle *dev, uint8_t reg, uint8_t val)
{
	unsigned char buf[4] = { 0x02, 0xf0, reg, val };
	int ret;

	ret = uvc_set_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x02 || buf[1] != 0xf0 || buf[2] != reg || buf[3] != val)
		fprintf(stderr, "write_reg(0x%x,0x%x): %02x %02x %02x %02x\n",
			reg, val, buf[0], buf[1], buf[2], buf[3]);
	return ret;
}

static int set_get_verify_a0(libusb_device_handle *dev, uint8_t val,
			     uint8_t retval)
{
	uint8_t buf[4] = { 0xa0, val, 0x00, 0x00 };
	int ret;

	ret = uvc_set_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	memset(buf, 0, sizeof buf);
	ret = uvc_get_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0xa0 || buf[1] != retval || buf[2] != 0x00) {
		fprintf(stderr, "response, should be a0 %02x 00: %02x %02x %02x\n",
			retval, buf[0], buf[1], buf[2]);
		return -1;
	}

	return 0;
}

#define CONTROL_SEL	11
#define DATA_SEL	12

static int setup_control(libusb_device_handle *devhandle, uint8_t a, size_t len)
{
	unsigned char control[16];
	int ret;

	/* prepare */
	memset(control, 0, sizeof control);
	control[1] = a; /* alternating, 0x81 or 0x41 */
	control[2] = 0x80;
	control[3] = 0x01;
	control[9] = len;
	ret = uvc_set_cur(devhandle, 0, XU_ENTITY, CONTROL_SEL, control,
			  sizeof control);
	if (ret < 0)
		return ret;

	usleep(20000);

	return 0;
}

static int radio_write(libusb_device_handle *devhandle, const uint8_t *buf, size_t len)
{
	unsigned char data[127];
	int ret;
	int i;

	if (len > 126)
		return -EINVAL;

	memset(data, 0, sizeof data);
	for (i = 0; i < len; i++) {
		data[i] = buf[i];
		data[126] -= buf[i]; /* calculate checksum */
	}

	ret = setup_control(devhandle, 0x81, sizeof data);
	if (ret < 0) return ret;

	/* send data */
	ret = uvc_set_cur(devhandle, 0, XU_ENTITY, DATA_SEL, data,
			  sizeof data);
	if (ret < 0)
		return ret;

	usleep(20000);

	ret = setup_control(devhandle, 0x41, sizeof data);
	if (ret < 0) return ret;

	/* expect all zeros */
	ret = uvc_get_cur(devhandle, 0, XU_ENTITY, DATA_SEL, data,
			  sizeof data);
	if (ret < 0)
		return ret;
#if 0
	for (i = 0; i < 127; i++)
		if (data[i])
			break;
	if (i != 127) {
		printf("not clear\n");
		for (i = 0; i < 127; i++)
			printf("%02x ", data[i]);
		printf("\n");
	}
#endif

	usleep(20000);

	ret = setup_control(devhandle, 0x81, sizeof data);
	if (ret < 0) return ret;

	/* clear */
	memset(data, 0, sizeof data);
	ret = uvc_set_cur(devhandle, 0, XU_ENTITY, DATA_SEL, data, sizeof data);
	if (ret < 0)
		return ret;

	usleep(20000);

	ret = setup_control(devhandle, 0x41, sizeof data);
	if (ret < 0) return ret;

	ret = uvc_get_cur(devhandle, 0, XU_ENTITY, DATA_SEL, data, sizeof data);
	if (ret < 0)
		return ret;
	for (i = 2; i < 126; i++)
		if (data[i])
			break;
	/* uint8_t chksum = 0 - data[0] - data[1]; */
	if (data[0] != buf[0] || data[1] != buf[1] /* || i != 126 ||
	    data[126] != chksum*/) {
		printf("unexpected read\n");
		for (i = 0; i < 127; i++)
			printf("%02x ", data[i]);
		printf("\n");
	}

	return 0;
}

int esp770u_setup_radio(libusb_device_handle *devhandle, uint32_t radio_id)
{
	int ret;

	const uint8_t buf1[7] = { 0x40, 0x10, radio_id & 0xff,
				  (radio_id >> 8) & 0xff,
				  (radio_id >> 16) & 0xff,
				  radio_id >> 24, 0x8c };
	ret = radio_write(devhandle, buf1, sizeof buf1);
	if (ret < 0)
		return ret;

	const uint8_t buf2[10] = { 0x50, 0x11, 0xf4, 0x01, 0x00, 0x00,
					       0x67, 0xff, 0xff, 0xff };
	ret = radio_write(devhandle, buf2, sizeof buf2);
	if (ret < 0)
		return ret;

	const uint8_t buf3[2] = { 0x61, 0x12 };
	ret = radio_write(devhandle, buf3, sizeof buf3);
	if (ret < 0)
		return ret;

	const uint8_t buf4[2] = { 0x71, 0x85 };
	ret = radio_write(devhandle, buf4, sizeof buf4);
	if (ret < 0)
		return ret;

	const uint8_t buf5[2] = { 0x81, 0x86 };
	return radio_write(devhandle, buf5, sizeof buf5);
}

/* Initial register setup, only after camera plugin */
int esp770u_init_regs(libusb_device_handle *devhandle)
{
	int ret;
	uint8_t val;

	ret = set_get_verify_a0(devhandle, 0x03, 0xb2);
	if (ret < 0) return ret;

	ret = esp770u_read_reg(devhandle, 0x5a, &val);
	if (ret < 0) return ret;
	if (val != 0x01 && val != 0x03) printf("unexpected 5a value: %02x\n", val);

	ret = esp770u_write_reg(devhandle, 0x5a, 0x01); /* &= 0x02? */
	if (ret < 0) return ret;
        ret = esp770u_read_reg(devhandle, 0x5a, &val);
	if (ret < 0) return ret;
	if (val != 0x01) printf("unexpected 5a value: %02x\n", val);

	ret = esp770u_read_reg(devhandle, 0x18, &val); /* |= 0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x18, 0x0f);
	if (ret < 0) return ret;
	ret = esp770u_read_reg(devhandle, 0x17, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x17, 0xed); /* |= 0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x17, 0xec); /* &= ~0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x18, 0x0e); /* &= ~0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_read_reg(devhandle, 0x14, &val);
	if (ret < 0) return ret;

#if 0
	for (int i = 0; i < 256; i += 1) {
		uint8_t rval;
		if (i % 16 == 0)
			printf("%02x: ", i);
		ret = esp770u_read_reg(devhandle, i, &rval);
		if (ret < 0) return ret;
		printf("%02x ", rval);
		if (i % 16 == 15)
			printf("\n");
	}

	printf("--------------------------------------\n");

	for (int i = 0; i < 256; i += 1) {
		uint8_t rval;
		if (i % 16 == 0)
			printf("%02x: ", i);
		ret = esp770u_read_reg_f1(devhandle, i, &rval);
		if (ret < 0) return ret;
		printf("%02x ", rval);
		if (i % 16 == 15)
			printf("\n");
	}
#endif

	return ret;
}
