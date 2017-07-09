#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <unistd.h>
#include <SDL.h>

#include "libuvc/libuvc.h"
#include "blobwatch.h"

#define ASSERT_MSG(_v, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); exit(1); }
#define WIDTH  1280
#define HEIGHT  720
#define FPS      55

#define SET_CUR 0x01
#define GET_CUR 0x81
#define TIMEOUT 1000

/* change this to the Rift HMD's radio id */
#define RIFT_RADIO_ID 0x12345678

int esp770u_read_reg2(libusb_device_handle *dev, uint8_t reg, uint16_t *val);

int uvc_set_cur(libusb_device_handle *dev, uint8_t interface, uint8_t entity,
		uint8_t selector, unsigned char *data, uint16_t wLength)
{
	uint8_t bmRequestType = LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = SET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | interface;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		fprintf(stderr, "failed to transfer SET CUR %u %u %u: %d %d %m\n",
				interface, entity, selector, ret, errno);
	}
	return ret;
}

int uvc_get_cur(libusb_device_handle *dev, uint8_t interface, uint8_t entity,
		uint8_t selector, unsigned char *data, uint16_t wLength)
{
	uint8_t bmRequestType = 0x80 | LIBUSB_REQUEST_TYPE_CLASS |
		LIBUSB_RECIPIENT_INTERFACE;
	uint8_t bRequest = GET_CUR;
	uint16_t wValue = selector << 8;
	uint16_t wIndex = entity << 8 | interface;
	unsigned int timeout = TIMEOUT;
	int ret;

	ret = libusb_control_transfer(dev, bmRequestType, bRequest, wValue,
			wIndex, data, wLength, timeout);
	if (ret < 0) {
		fprintf(stderr, "failed to transfer GET CUR %u %u %u: %d %d %m\n",
				interface, entity, selector, ret, errno);
	}
	return ret;
}

typedef struct
{
	SDL_Surface* target;
	SDL_mutex* mutex;
	struct blobservation* bwobs;
	libusb_device_handle *usb_devh;
} cb_data;

struct blobwatch* bw;

#define min(a,b) ((a) < (b) ? (a) : (b))

void cb(uvc_frame_t *frame, void *ptr)
{
	if(frame->data_bytes != WIDTH * HEIGHT){
		printf("bad frame: %d\n", (int)frame->data_bytes);
	//	return;
	}

	cb_data* data = (cb_data*)ptr;
			
	SDL_LockMutex(data->mutex);
	SDL_LockSurface(data->target);

	unsigned char* spx = frame->data;
	unsigned char* tpx = data->target->pixels;

	for(int i = 0; i < min(frame->data_bytes, WIDTH * HEIGHT); i++)
	{
		(*tpx++) = *spx;
		(*tpx++) = *spx;
		(*tpx++) = *spx;
		(*tpx++) = *(spx++);
	}

	blobwatch_process(bw, frame->data, WIDTH, HEIGHT, 0, NULL, &data->bwobs);

	if (data->bwobs)
	{
		if (data->bwobs->num_blobs > 0)
		{
			printf("Blobs: %d\n", data->bwobs->num_blobs);

			for (int index = 0; index < data->bwobs->num_blobs; index++)
			{
				printf("Blob[%d]: %d,%d\n",
					index,
					data->bwobs->blobs[index].x,
					data->bwobs->blobs[index].y);
			}
		}
	}

	SDL_UnlockSurface(data->target);

#if 0
	int ret;
	uint16_t val;
	libusb_device_handle *devh = data->usb_devh;
	esp770u_read_reg2(devh, 0xb2, &val);
	printf("b2: %04x\n", val);
#endif

	SDL_UnlockMutex(data->mutex);
}

#define CONTROL_IFACE   0

#define XU_ENTITY       4

#define REG_SEL         3

int esp770u_read_reg(libusb_device_handle *dev, uint8_t reg, uint8_t *val)
{
	unsigned char buf[4] = { 0x82, 0xf0, reg };
	int ret;
	uint8_t interface = CONTROL_IFACE;
	uint8_t entity = XU_ENTITY;

	ret = uvc_set_cur(dev, interface, entity, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, interface, entity, REG_SEL, buf, 3);
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
	uint8_t interface = CONTROL_IFACE;
	uint8_t entity = XU_ENTITY;

	ret = uvc_set_cur(dev, interface, entity, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, interface, entity, REG_SEL, buf, 3);
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
	uint8_t interface = CONTROL_IFACE;
	uint8_t entity = XU_ENTITY;

	ret = uvc_set_cur(dev, interface, entity, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, interface, entity, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x02 || buf[1] != 0xf0 || buf[2] != reg || buf[3] != val)
		fprintf(stderr, "write_reg(0x%x,0x%x): %02x %02x %02x %02x\n",
			reg, val, buf[0], buf[1], buf[2], buf[3]);
	return ret;
}

#define REG_SEL2 2

int esp770u_read_reg2(libusb_device_handle *dev, uint8_t reg, uint16_t *val)
{
	unsigned char buf[6] = { 0x86, 0x20, 0x30, reg, 0x00, 0x00 };
	int ret;
	uint8_t interface = CONTROL_IFACE;
	uint8_t entity = XU_ENTITY;

	ret = uvc_set_cur(dev, interface, entity, REG_SEL2, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, interface, entity, REG_SEL2, buf, 3);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x86)
		fprintf(stderr, "read_reg(0x%x): %02x %02x %02x\n",
			reg, buf[0], buf[1], buf[2]);
	*val = (buf[2] << 8) | buf[1];
	return ret;
}

int esp770u_write_reg2(libusb_device_handle *dev, uint8_t reg, uint8_t val1, uint8_t val2)
{
	unsigned char buf[64] = { 0x06, 0x20, 0x30, reg, val1, val2 };
	int ret;
	uint8_t interface = CONTROL_IFACE;
	uint8_t entity = XU_ENTITY;

	ret = uvc_set_cur(dev, interface, entity, REG_SEL2, buf, 6);
	if (ret < 0)
		return ret;
	ret = uvc_get_cur(dev, interface, entity, REG_SEL2, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x06 || buf[1] != 0x20 || buf[2] != 0x30 || buf[3] != reg ||
	    buf[4] != val1 || buf[5] != val2)
		fprintf(stderr, "write_reg(0x%x,0x%x,0x%x): %02x %02x %02x %02x\n",
			reg, val1, val2, buf[0], buf[1], buf[2], buf[3]);
	return ret;
}

static int set_get_verify_a0(libusb_device_handle *dev, uint8_t val,
			     uint8_t retval)
{
	uint8_t buf[4] = { 0xa0, val, 0x00, 0x00 };
	int ret;

	ret = uvc_set_cur(dev, 0, 4, 3, buf, sizeof buf);
	if (ret < 0)
		return ret;
	memset(buf, 0, sizeof buf);
	ret = uvc_get_cur(dev, 0, 4, 3, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0xa0 || buf[1] != retval || buf[2] != 0x00) {
		printf("response, should be a0 %02x 00: %02x %02x %02x\n",
				retval, buf[0], buf[1], buf[2]);
		return -1;
	}

	return 0;
}

#define CONTROL_SEL	11
#define DATA_SEL	12

static int setup_control(libusb_device_handle *devhandle, uint8_t a, size_t len)
{
	const uint8_t interface = CONTROL_IFACE;
	const uint8_t entity = XU_ENTITY;
	unsigned char control[16];
	int ret;

	/* prepare */
	memset(control, 0, sizeof control);
	control[1] = a; /* 0x81 or 0x41 */
	control[2] = 0x80;
	control[3] = 0x01;
	control[9] = len;
	ret = uvc_set_cur(devhandle, interface, entity, CONTROL_SEL, control,
			  sizeof control);
	if (ret < 0)
		return ret;

	usleep(20000);
}

static int radio_write(libusb_device_handle *devhandle, const uint8_t *buf, size_t len)
{
	const uint8_t interface = CONTROL_IFACE;
	const uint8_t entity = XU_ENTITY;
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
	ret = uvc_set_cur(devhandle, interface, entity, DATA_SEL, data,
			  sizeof data);
	if (ret < 0)
		return ret;

	usleep(20000);

	ret = setup_control(devhandle, 0x41, sizeof data);
	if (ret < 0) return ret;

	/* expect all zeros */
	ret = uvc_get_cur(devhandle, interface, entity, DATA_SEL, data,
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
	ret = uvc_set_cur(devhandle, interface, entity, DATA_SEL, data,
			  sizeof data);
	if (ret < 0)
		return ret;

	usleep(20000);

	ret = setup_control(devhandle, 0x41, sizeof data);
	if (ret < 0) return ret;

	ret = uvc_get_cur(devhandle, interface, entity, DATA_SEL, data,
			  sizeof data);
	if (ret < 0)
		return ret;
	for (i = 2; i < 126; i++)
		if (data[i])
			break;
	uint8_t chksum = 0 - data[0] - data[1];
	if (data[0] != buf[0] || data[1] != buf[1] /* || i != 126 ||
	    data[126] != chksum*/) {
		printf("unexpected read\n");
		for (i = 0; i < 127; i++)
			printf("%02x ", data[i]);
		printf("\n");
	}

	return 0;
}

static int setup_radio(libusb_device_handle *devhandle)
{
	int ret;

	const uint8_t buf1[7] = { 0x40, 0x10, RIFT_RADIO_ID & 0xff,
				  (RIFT_RADIO_ID >> 8) & 0xff,
				  (RIFT_RADIO_ID >> 16) & 0xff,
				  (RIFT_RADIO_ID >> 24) & 0xff, 0x8c };
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
	ret = radio_write(devhandle, buf5, sizeof buf5);
	if (ret < 0)
		return ret;
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

/* Sensor setup after stream start */
int esp770u_init_regs2(libusb_device_handle *devh, cb_data *data)
{
	uint16_t val;
	int ret;

#if 1
	SDL_LockMutex(data->mutex);
	for (int i = 0; i < 256; i += 2) {
		if (i % 16 == 0)
			printf("%02x: ", i);
		ret = esp770u_read_reg2(devh, i, &val);
		if (ret < 0) return ret;
		printf("%04x ", val);
		if (i % 16 == 14)
			printf("\n");
	}
	SDL_UnlockMutex(data->mutex);
#endif

	ret = esp770u_read_reg2(devh, 0x00, &val);
	if (ret < 0) return ret;
	if (val != 0x2406) printf("00 != 2406: %04x\n", val);

	ret = esp770u_read_reg2(devh, 0x00, &val);
	if (ret < 0) return ret;
	if (val != 0x2406) printf("00 != 2406\n");

	ret = esp770u_read_reg2(devh, 0x00, &val);
	if (ret < 0) return ret;
	if (val != 0x2406) printf("00 != 2406\n");

	ret = esp770u_read_reg2(devh, 0x0e, &val);
	if (ret < 0) return ret;
	if (val != 0x1300) printf("0e != 1300\n");

	ret = esp770u_read_reg2(devh, 0xb0, &val);
	if (ret < 0) return ret;
	if (val != 0x0080) printf("b0 != 0080\n");

	ret = esp770u_read_reg2(devh, 0x64, &val);
	if (ret < 0) return ret;
	if (val != 0x1882) printf("64 != 1882\n");

	ret = esp770u_write_reg2(devh, 0x64, 0x19, 0x82);
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x1e, 0x00, 0x00); // darkens the picture
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x32, 0x00, 0x00);
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x5e, 0x00, 0x20); // 00 07 - darkens the picture
	if (ret < 0) return ret;

	ret = esp770u_read_reg2(devh, 0xb0, &val);
	if (ret < 0) return ret;
	if (val != 0x0080) printf("b0 != 0080\n");

	ret = esp770u_write_reg2(devh, 0xb0, 0x00, 0x80);
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x02, 0x00, 0x00);
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x06, 0x03, 0xbf); // 959
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x04, 0x00, 0x00);
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x08, 0x04, 0xff); // 1279
	if (ret < 0) return ret;
#if 0
	ret = esp770u_write_reg2(devh, 0x0c, 0x05, 0x6c); // 1388: causes hsync loss (buffer overflows?)
	if (ret < 0) return ret;
#endif

	ret = esp770u_read_reg2(devh, 0xb0, &val);
	if (ret < 0) return ret;
	if (val != 0x0080) printf("b0 != 0080\n");

	ret = esp770u_write_reg2(devh, 0xb0, 0x04, 0x80); // 1152
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x0a, 0x03, 0xe5); // 997: darkens the picture
	if (ret < 0) return ret;

	ret = esp770u_read_reg2(devh, 0x0c, &val);
	if (ret < 0) return ret;
	if (val != 0x056c) printf("b0 != 056c\n");

	ret = esp770u_write_reg2(devh, 0x12, 0x00, 0x1a);
	if (ret < 0) return ret;
	ret = esp770u_write_reg2(devh, 0x14, 0x02, 0x86);
	if (ret < 0) return ret;

	ret = esp770u_read_reg2(devh, 0x12, &val);
	if (ret < 0) return ret;
	if (val != 0x001a) printf("b0 != 001a\n");
	ret = esp770u_read_reg2(devh, 0x14, &val);
	if (ret < 0) return ret;
	if (val != 0x0286) printf("b0 != 0286\n");
	ret = esp770u_read_reg2(devh, 0x1a, &val);
	if (ret < 0) return ret;
	if (val != 0x10dc) printf("b0 != 10dc\n");
	/* 1a: 10 dc -> 19 d8 - start external exposure control via nRF51288 */
	ret = esp770u_write_reg2(devh, 0x1a, 0x19, 0xd8);
	if (ret < 0) return ret;

	/* Enable the b2 readback */
	ret = esp770u_read_reg2(devh, 0xb4, &val);
	if (ret < 0) return ret;
	if (val != 0x0000) printf("b4 != 0000\n");
	ret = esp770u_write_reg2(devh, 0xb4, 0x00, 0x11);
	if (ret < 0) return ret;

	ret = esp770u_read_reg2(devh, 0xc6, &val);
	if (ret < 0) return ret;
	if (val != 0x01d0 && val != 0x01d && val != 0x1ce)
		printf("b0 != 01d0/01d2: %04x\n", val);

	ret = esp770u_read_reg2(devh, 0xc8, &val);
	if (ret < 0) return ret;
	if (val != 0x01ba && val != 0x01bc && val != 0x01bb)
		printf("b0 != 01ba/01bc: %04x\n", val);
}

int main(int argc, char** argv)
{
    uvc_error_t res;
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_stream_ctrl_t ctrl;
    uvc_device_handle_t *devh;
    struct libusb_device_handle *usb_devh;
    int ret;

    bw = blobwatch_new(WIDTH, HEIGHT);

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_mutex* mutex = SDL_CreateMutex();

    res = uvc_init(&ctx, NULL);
    ASSERT_MSG(res >= 0, "could not initalize libuvc\n");

    res = uvc_find_device(ctx, &dev, 0x2833, 0, NULL);
    ASSERT_MSG(res >= 0, "could not find the camera\n");

    res = uvc_open(dev, &devh);
    ASSERT_MSG(res >= 0, "could not open the camera\n");

    usb_devh = uvc_get_libusb_handle(devh);

    SDL_Window* window = SDL_CreateWindow("Playground", SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, 0);

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    res = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_ANY,
            WIDTH / 2, HEIGHT, FPS);
    ASSERT_MSG(res >= 0, "could not get format size\n");

    uvc_print_diag(devh, stderr);

    SDL_Surface* target = SDL_CreateRGBSurface(
            SDL_SWSURFACE, WIDTH, HEIGHT, 32, 0xff, 0xff00, 0xff0000, 0);

    cb_data data = { target, mutex, NULL, usb_devh };

    ret = esp770u_init_regs(usb_devh);
    ASSERT_MSG(ret >= 0, "could not init eSP770u\n");

    res = uvc_start_streaming(devh, &ctrl, cb, &data, 0);
    ASSERT_MSG(res >= 0, "could not start streaming\n");

    bool done = false;

    while(!done)
    {
        SDL_Event event;
        while(SDL_PollEvent(&event))
        {
            if(event.type == SDL_QUIT ||
               (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE))
            {
                done = true;
            }
            if(event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE)
            {
                ret = esp770u_init_regs2(usb_devh, &data);
                if (ret < 0)
                    return ret;

                ret = setup_radio(usb_devh);
                if (ret < 0)
                    return ret;
            }
        }

        SDL_RenderClear(renderer);

        SDL_LockMutex(mutex);
        SDL_Texture* tex = SDL_CreateTextureFromSurface(renderer, target);
        SDL_UnlockMutex(mutex);

        SDL_RenderCopy(renderer, tex, NULL, NULL);

        if (data.bwobs)
        {
            for (int index = 0; index < data.bwobs->num_blobs; index++)
            {
                struct blob* blob = &data.bwobs->blobs[index];
                SDL_Rect rect = {blob->x, blob->y, 20, 20};
                SDL_SetRenderDrawColor(renderer, 255, 0, 0, 128);
                SDL_RenderDrawRect(renderer, &rect);
            }
        }

        SDL_DestroyTexture(tex);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyMutex(mutex);
    SDL_Quit();

    return 0;
}
