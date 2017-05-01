#include "Window.h"
#include <SDL2/SDL_ttf.h>
#include <ros/ros.h>

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 368
#define WINDOW_WIDTH 750
#define WINDOW_HEIGHT 400

Window window;

GUIC* wifi;
GUIC* batt;

GUIC* lat;
GUIC* lon;
GUIC* alt;

GUIC* velx;
GUIC* vely;
GUIC* velz;


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
	if(!alive) return;

	if(event->type == SDL_QUIT) {destroy();} else if(event->type == SDL_MOUSEBUTTONDOWN) {}
}

void Window::updateVideoTexture(const sensor_msgs::ImageConstPtr& img) {
	// ROS_INFO("GOT IMAGE WITH FORMAT: %s, size (%d, %d)", img->encoding.c_str(), img->width, img->height);


	bool bgr = strncmp(img->encoding.c_str(), "bgr", 3) == 0;

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
			if(bgr) {
				pixels[texIndex + 0] = img->data[iIndex + 0];	// red
				pixels[texIndex + 1] = img->data[iIndex + 1];	// green
				pixels[texIndex + 2] = img->data[iIndex + 2];	// blue
			} else {
				pixels[texIndex + 0] = img->data[iIndex + 2];	// red
				pixels[texIndex + 1] = img->data[iIndex + 1];	// green
				pixels[texIndex + 2] = img->data[iIndex + 0];	// blue
			}
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
		r.y = WINDOW_HEIGHT - VIDEO_HEIGHT;

		SDL_RenderCopy(ren, video, NULL, &r);

		// SDL_RenderCopy(ren, video, NULL, NULL);

		video_dirty = false;
	}

	wifi->render(ren);
	batt->render(ren);

	// FUCK THIS THING. SOLID 3 hours GOOONNNNEEE
	SDL_RenderPresent(ren);
}

bool Window::ok() {
	return alive;
}

void Window::destroy() {
	alive = false;


// seg fault in closefont... no idea why
	// TTF_CloseFont(font);
	SDL_DestroyRenderer(ren);
	SDL_DestroyWindow(win);

	TTF_Quit();
	SDL_Quit();
}

void Window::makeGUI() {
	// GUIC* text = new GUIC(font, 5, 4, -1, 28);
	// text->setText("test text", ren);


	batt = new GUIC(font, 4, 4, 100, 28);

	batt->setText("BAT: 90%", ren);

	wifi = new GUIC(font, 200, 4, 50, 28);

	wifi->setText("WIFI: 10%", ren);


	//
	// lat;
	// lon;
	// alt;
	//
	// velx;
	// vely;
	// velz;
}

bool Window::init() {
	alive = false;
	if(SDL_Init(SDL_INIT_VIDEO) < 0) {
		ROS_ERROR( "SDL INIT FAIL: %s\n", SDL_GetError() );
		return true;
	}

	alive = true;
	if(TTF_Init() < 0) {
		ROS_ERROR( "TTF INIT FAIL: %s\n", TTF_GetError() );
		return true;
	}

	font = TTF_OpenFont("/home/michionlion/catkin_ws/src/bebop_teleop/sqr.ttf", 28);
	if(font == NULL) ROS_ERROR( "TTF FONT LOAD FAIL: %s\n", TTF_GetError() );

	// return true;

	SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_OPENGL, &win, &ren);
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
	makeGUI();
	return false;
}
