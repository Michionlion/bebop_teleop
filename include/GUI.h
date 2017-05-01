#ifndef __GUI_H__
#define __GUI_H__
#include "Callbacks.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>

class GUIC {
public:
	GUIC(TTF_Font*, int, int, int, int);
	~GUIC(void);

	void setText(std::string, SDL_Renderer*);
	std::string getText(void) const;
	void setTexture(SDL_Texture*);
	void render(SDL_Renderer*);
	bool inside(int, int) const;
	void callCB(void);
	void setCallback(Callback*);
	void destroy(void);
	SDL_Rect* getBounds(void);

private:
	SDL_Rect bounds = {0, 0, 0, 0};
	SDL_Texture* texture = NULL;
	Callback* callback = NULL;
	std::string text;
	TTF_Font* font;
	bool dirty = false;
};

#endif
