#ifndef __GUI_H__
#define __GUI_H__
#include "Callbacks.h"
#include <SDL2/SDL.h>
#include <string>

class GUIC {
public:
	GUIC(int, int, int, int);
	~GUIC(void);

	void setText(std::string);
	std::string getText(void) const;
	void setTexture(SDL_Texture*);
	void render(SDL_Renderer*) const;
	bool inside(int, int) const;
	void callCB(void);
	void setCallback(Callback*);

private:
	SDL_Rect bounds;
	SDL_Texture* texture;
	Callback* callback;
	std::string text;
	bool dirty;
};

#endif
