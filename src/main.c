#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <unistd.h>
#include <SDL.h>

#include "blobwatch.h"
#include "ar0134.h"
#include "esp770u.h"
#include "mt9v034.h"
#include "uvc.h"

#define ASSERT_MSG(_v, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); exit(1); }

/* change this to the Rift HMD's radio id */
#define RIFT_RADIO_ID 0x12345678

#define NUM_PATTERNS_CV1 34
#define NUM_PATTERNS_DK2 40

static const uint16_t patterns[40] = {
	0x001, 0x006, 0x01a, 0x01d, 0x028, 0x02f, 0x033, 0x04b, 0x04c, 0x057,
	0x062, 0x065, 0x079, 0x07e, 0x090, 0x0a4, 0x114, 0x151, 0x183, 0x18c,
	0x199, 0x1aa, 0x1b5, 0x1c0, 0x1cf, 0x1d6, 0x1e9, 0x1f3, 0x1fc, 0x230,
	0x252, 0x282, 0x285, 0x29b, 0x29c, 0x2ae, 0x2b7, 0x2c8, 0x2d1, 0x2e3,
};

typedef struct
{
	SDL_Surface* target;
	SDL_mutex* mutex;
	struct blobservation* bwobs;
	libusb_device_handle *usb_devh;
	int num_patterns;
	const uint16_t *patterns;
} cb_data;

struct blobwatch* bw;

#define min(a,b) ((a) < (b) ? (a) : (b))

void cb(struct uvc_stream *stream)
{
	int width = stream->width;
	int height = stream->height;

	if(stream->payload_size != width * height) {
		printf("bad frame: %d\n", (int)stream->payload_size);
	//	return;
	}

	cb_data* data = stream->user_data;
	if (!data)
		return;

	SDL_LockMutex(data->mutex);
	SDL_LockSurface(data->target);

	unsigned char* spx = stream->frame;
	unsigned char* tpx = data->target->pixels;

	for(int i = 0; i < min(stream->payload_size, width * height); i++)
	{
		(*tpx++) = *spx;
		(*tpx++) = *spx;
		(*tpx++) = *spx;
		(*tpx++) = *(spx++);
	}

	blobwatch_process(bw, stream->frame, width, height, 0,
			  data->num_patterns, data->patterns, &data->bwobs);

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
    libusb_context *ctx;
    libusb_device_handle *usb_devh;
    struct uvc_stream stream;
    int ret;
    uint16_t pid;

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_mutex* mutex = SDL_CreateMutex();

    ret = libusb_init(&ctx);
    ASSERT_MSG(ret >= 0, "could not initalize libusb\n");

    usb_devh = libusb_open_device_with_vid_pid(ctx, 0x2833, pid = CV1_PID);
    if (!usb_devh)
	usb_devh = libusb_open_device_with_vid_pid(ctx, 0x2833, pid = DK2_PID);
    ASSERT_MSG(usb_devh, "could not find or open the camera\n");

    memset(&stream, 0, sizeof(stream));
    stream.frame_cb = cb;
    ret = uvc_stream_start(ctx, usb_devh, &stream);
    ASSERT_MSG(ret >= 0, "could not start streaming\n");

    bw = blobwatch_new(stream.width, stream.height);

    SDL_Window* window = SDL_CreateWindow("Playground", SDL_WINDOWPOS_UNDEFINED,
            SDL_WINDOWPOS_UNDEFINED, stream.width, stream.height, 0);

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    SDL_Surface* target = SDL_CreateRGBSurface(
            SDL_SWSURFACE, stream.width, stream.height, 32, 0xff, 0xff00, 0xff0000, 0);

    cb_data data = { target, mutex, NULL, usb_devh,
            (pid == CV1_PID) ? NUM_PATTERNS_CV1 : NUM_PATTERNS_DK2, patterns };

    stream.user_data = &data;

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
                if (pid == CV1_PID) {
                    ret = ar0134_init(usb_devh);
                    if (ret < 0)
                        return ret;

                    ret = esp770u_setup_radio(usb_devh, RIFT_RADIO_ID);
                    if (ret < 0)
                        return ret;
                } else if (pid == DK2_PID) {
                    ret = mt9v034_setup(usb_devh);
                    if (ret < 0)
                        return ret;

                    ret = mt9v034_set_sync(usb_devh, true);
                    if (ret < 0)
                        return ret;
                }
                blobwatch_set_flicker(true);
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
                if (blob->led_id != -1 && blob->age >= 10)
                    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 128);
                else
                    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 128);
                SDL_RenderDrawRect(renderer, &rect);
            }
        }

        SDL_DestroyTexture(tex);

        SDL_RenderPresent(renderer);
    }

    uvc_stream_stop(&stream);

    SDL_DestroyMutex(mutex);
    SDL_Quit();

    return 0;
}
