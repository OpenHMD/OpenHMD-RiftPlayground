#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <unistd.h>
#include <SDL.h>

#include "libuvc/libuvc.h"
#include "blobwatch.h"
#include "ar0134.h"
#include "esp770u.h"
#include "uvc.h"

#define ASSERT_MSG(_v, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); exit(1); }
#define WIDTH  1280
#define HEIGHT  720
#define FPS      55

/* change this to the Rift HMD's radio id */
#define RIFT_RADIO_ID 0x12345678

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
	SDL_UnlockMutex(data->mutex);
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
                ret = ar0134_init(usb_devh);
                if (ret < 0)
                    return ret;

                ret = esp770u_setup_radio(usb_devh, RIFT_RADIO_ID);
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
                SDL_Rect rect = {blob->x - 10, blob->y - 10, 20, 20};
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
