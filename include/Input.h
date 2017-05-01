#ifndef __INPUT_H__
#define __INPUT_H__

#include "Events.h"
#include <SDL2/SDL.h>
#include <vector>

class KeyListener {
public:
	virtual void key(SDL_KeyboardEvent*) = 0;
};

class Input : Listener {
public:
	Input(void);
	void event(SDL_Event*);

	bool isKeyDown(SDL_Scancode);
	void registerKeyListener(KeyListener*);
	bool unregisterKeyListener(KeyListener*);

private:
	std::vector<KeyListener*> keyListeners;
	const Uint8* keysDown;
};

extern Input* input;

#endif
