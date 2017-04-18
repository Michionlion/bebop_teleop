#include "InputWindow.h"
#include <ros/ros.h>

InputWindow::InputWindow(bool& err) {
	if(SDL_Init(SDL_INIT_VIDEO) < 0) {
		ROS_ERROR( "SDL INIT FAIL: %s\n", SDL_GetError() );
		err = true;
		return;
	}

	// SDL_EnableKeyRepeat( 0, SDL_DEFAULT_REPEAT_INTERVAL );
	// SDL 1.2
	// SDL_WM_SetCaption("Bebop_Teleop keyboard input", NULL);
	// window = SDL_SetVideoMode(200, 200, 0, 0);
	SDL_CreateWindowAndRenderer(200, 200, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE, &window, &render);
	if(window == NULL || render == NULL) {
		ROS_ERROR( "SDL CREATE WINDOW FAIL: %s\n", SDL_GetError() );
		err = true;
		return;
	}

	err = false;

	registerEventCallback(this);
}

InputWindow::~InputWindow(void) {
	SDL_DestroyRenderer(this->render);
	SDL_DestroyWindow(this->window);
	SDL_Quit();
}

void InputWindow::event(SDL_Event* event) {
	ROS_INFO("EVENT");
	switch(event->type) {
	case SDL_KEYUP:

		break;

	case SDL_KEYDOWN:
		if(event->key.repeat == 0) {}
		break;
	}
}

SDL_Window* InputWindow::getWindow() {
	return this->window;
}

SDL_Renderer* InputWindow::getRender() {
	return this->render;
}
