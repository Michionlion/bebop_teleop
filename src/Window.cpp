#include "Window.h"
#include <ros/ros.h>

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
	if(event->type == SDL_QUIT) {
		ros::shutdown();
		destroy();
	}
}

void Window::destroy() {
	SDL_DestroyRenderer(this->ren);
	SDL_DestroyWindow(this->win);
	SDL_Quit();
}

SDL_Window* Window::getWindow() {
	return this->win;
}

SDL_Renderer* Window::getRender() {
	return this->ren;
}

bool Window::init() {
	if(SDL_Init(SDL_INIT_VIDEO) < 0) {
		ROS_ERROR( "SDL INIT FAIL: %s\n", SDL_GetError() );
		return true;
	}
	SDL_CreateWindowAndRenderer(640, 400, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE, &win, &ren);
	if(win == NULL || ren == NULL) {
		ROS_ERROR( "SDL CREATE WINDOW FAIL: %s\n", SDL_GetError() );
		return true;
	}

	registerEventListener(this);
	SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
	SDL_RenderClear(ren);
	SDL_RenderPresent(ren);
	return false;
}
