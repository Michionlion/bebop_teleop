#include "Window.h"
#include <ros/ros.h>

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 368

Window window;

Window::Window(bool& err) {
	err = init();
}

Window::Window() {
	init();
}

Window::~Window() {
	destroy();
}

void Window::event(SDL_Event* event) {
	if(event->type == SDL_QUIT) destroy();
}

void Window::updateVideoTexture(const sensor_msgs::ImageConstPtr& img) {
	// ROS_INFO("GOT IMAGE WITH FORMAT: %s, size (%d, %d)", img->encoding.c_str(), img->width, img->height);

	int pitch = img->step;

	char* pixels;
	int texPitch;
	SDL_LockTexture(video, NULL, (void**) &pixels, &texPitch);

	if(pixels == NULL) goto END;

	int texIndex;
	int iIndex;
	int iCol;
	for(int row = 0; row < VIDEO_HEIGHT; row++) {
		iCol = 0;
		for(int col = 0; col + 3 < texPitch; col += 4) {
			if(iCol + 2 >= pitch) continue;
			texIndex = row * texPitch + col;
			iIndex = row * pitch + iCol;

			pixels[texIndex + 0] = img->data[iIndex + 2];	// red
			pixels[texIndex + 1] = img->data[iIndex + 1];	// green
			pixels[texIndex + 2] = img->data[iIndex + 0];	// blue
			pixels[texIndex + 3] = 0xFF;// alpha - doesn't matter unless blending
			iCol += 3;
		}
	}
	video_dirty = true;
END:
	SDL_UnlockTexture(video);
}

SDL_Rect r = {0, 0, VIDEO_WIDTH, VIDEO_HEIGHT};
void Window::update() {
	// do update
	if( !ok() ) return;

	if(video_dirty) {
		// center image in pane
		SDL_GetWindowSize(win, &r.x, NULL);
		r.x = (r.x / 2) - (VIDEO_WIDTH / 2);

		SDL_RenderCopy(ren, video, NULL, &r);

		// SDL_RenderCopy(ren, video, NULL, NULL);
		SDL_RenderPresent(ren);	// will want to move this out of if statement if anything else is being drawn/updated
		video_dirty = false;
	}

	// ROS_INFO("RENDER PRESENT");
}

bool Window::ok() {
	return alive;
}

void Window::destroy() {
	alive = false;
	SDL_DestroyRenderer(ren);
	SDL_DestroyWindow(win);
	SDL_Quit();
}

bool Window::init() {
	alive = false;
	if(SDL_Init(SDL_INIT_VIDEO) < 0) {
		ROS_ERROR( "SDL INIT FAIL: %s\n", SDL_GetError() );
		return true;
	}
	alive = true;

	SDL_CreateWindowAndRenderer(VIDEO_WIDTH + 200, VIDEO_HEIGHT + 80, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE, &win, &ren);
	if(win == NULL || ren == NULL) {
		ROS_ERROR( "SDL CREATE WINDOW FAIL: %s\n", SDL_GetError() );
		return true;
	}


	video = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB888, SDL_TEXTUREACCESS_STREAMING, VIDEO_WIDTH, VIDEO_HEIGHT);
	if(video == NULL) {
		ROS_ERROR( "SDL CREATE TEXTURE FAIL: %s\n", SDL_GetError() );
		return true;
	}
	registerEventListener(this);
	SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
	SDL_RenderClear(ren);
	SDL_RenderPresent(ren);
	return false;
}
