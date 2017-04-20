#include "Input.h"
#include <ros/ros.h>

Input input;

Input::Input() {
	registerEventListener(this);
	keysDown = SDL_GetKeyboardState(NULL);
}

void Input::event(SDL_Event* event) {
	switch(event->type) {
	case SDL_KEYUP:
	case SDL_KEYDOWN:
		if(event->key.repeat < 1) {
			ROS_INFO("KEYEVENT");
			for(auto & el : keyListeners) el->key(&event->key);
		}
		break;
	}
}

bool Input::isKeyDown(SDL_Scancode code) {
	return keysDown[code];
}

void Input::registerKeyListener(KeyListener* lis) {
	keyListeners.push_back(lis);
}

bool Input::unregisterKeyListener(KeyListener* lis) {
	auto beg = keyListeners.begin();
	auto end = keyListeners.end();
	for(; beg != end; ++beg) {
		if( (*beg) == lis ) {
			keyListeners.erase(beg);
			return true;
		}
	}
	return false;
}
