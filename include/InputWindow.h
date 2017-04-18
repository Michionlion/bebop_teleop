#ifndef __INPUT_H__
#define __INPUT_H__

#include "Events.h"
#include <SDL2/SDL.h>
#include <vector>

class KeyListener {
public:
	virtual void key(SDL_KeyboardEvent*);
};

class InputWindow : Listener {
public:
	InputWindow(bool& err);
	~InputWindow(void);

	void event(SDL_Event*);
	SDL_Window* getWindow(void);
	SDL_Renderer* getRender(void);

	bool isKeyDown(SDL_Scancode);
	void registerKeyListener(KeyListener*);
	bool unregisterKeyListener(KeyListener*);

private:
	// SDL_Surface* window;
	SDL_Window* window;
	SDL_Renderer* render;
	std::vector<KeyListener*> keyListeners;
	const Uint8* keysDown;
};

extern InputWindow* input;

#endif
