#include <SDL2/SDL.h>

class InputWindow {
public:
	InputWindow(bool& err);
	~InputWindow(void);

	bool get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers);
	SDL_Window* getWindow();
	SDL_Renderer* getRender();

private:
	// SDL_Surface* window;
	SDL_Window* window;
	SDL_Renderer* render;
};
