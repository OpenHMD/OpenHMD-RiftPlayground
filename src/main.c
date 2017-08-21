#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <unistd.h>
#include <SDL.h>

#include "blobwatch.h"
#include "ar0134.h"
#include "esp770u.h"
#include "uvc.h"

#define ASSERT_MSG(_v, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); exit(1); }

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

	blobwatch_process(bw, stream->frame, width, height, 0, NULL, &data->bwobs);

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

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_mutex* mutex = SDL_CreateMutex();

    ret = libusb_init(&ctx);
    ASSERT_MSG(ret >= 0, "could not initalize libusb\n");

    usb_devh = libusb_open_device_with_vid_pid(ctx, 0x2833, CV1_PID);
    if (!usb_devh)
	usb_devh = libusb_open_device_with_vid_pid(ctx, 0x2833, DK2_PID);
    ASSERT_MSG(usb_devh, "could not find or open the camera\n");

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

    cb_data data = { target, mutex, NULL, usb_devh };
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

    uvc_stream_stop(&stream);

    SDL_DestroyMutex(mutex);
    SDL_Quit();

    return 0;
}
