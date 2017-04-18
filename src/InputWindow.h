#include "Events.h"
#include <SDL2/SDL.h>
#include <vector>

// Listener points to a function that accepts a keyboard event and returns void
typedef void (*KeyListener)(SDL_KeyboardEvent*);

class InputWindow : Listener {
public:
	InputWindow(bool& err);
	~InputWindow(void);

	void event(SDL_Event*);
	SDL_Window* getWindow(void);
	SDL_Renderer* getRender(void);

	bool isKeyDown(SDL_Scancode);
	void registerListener(KeyListener);
private:
	// SDL_Surface* window;
	SDL_Window* window;
	SDL_Renderer* render;
	std::vector<KeyListener> listeners;
	std::vector<SDL_Scancode> keysDown;
};
