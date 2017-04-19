#ifndef __WINDOW_H__
#define __WINDOW_H__

#include "Events.h"
#include <SDL2/SDL.h>

class Window : Listener {
public:
	Window(bool&);
	Window(void);
	~Window(void);
	void event(SDL_Event*);
	void destroy(void);
	SDL_Window* getWindow(void);
	SDL_Renderer* getRender(void);

private:
	SDL_Window* win;
	SDL_Renderer* ren;
	bool init(void);

	// Window(const Window&);
};

extern Window window;

#endif
