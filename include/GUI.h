#ifndef __GUI_H__
#define __GUI_H__
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <functional>
#include <string>
class GUIC {
public:
	GUIC(TTF_Font*, int, int, int, int);
	~GUIC(void);

	void setText(std::string, SDL_Renderer*, short);
	void setText(std::string, SDL_Renderer*);
	std::string getText(void) const;
	void setTexture(SDL_Texture*);
	SDL_Texture* getTexture(void) const;
	void render(SDL_Renderer*);
	bool inside(int, int) const;
	void callCB(void);
	void setCallback(std::function<void(GUIC*)>);
	void destroy(void);
	SDL_Rect* getBounds(void);

	void setFG(uint8_t, uint8_t, uint8_t);
	void setBG(uint8_t, uint8_t, uint8_t);

	static const short RESIZE_X = 1;
	static const short RESIZE_Y = 2;
	static const short RESIZE_NONE = 0;
private:
	SDL_Rect bounds = {0, 0, 0, 0};
	SDL_Color fg = {255, 255, 255, 255};
	SDL_Color bg = {  0, 0, 0, 255};
	SDL_Texture* texture = NULL;
	std::function<void(GUIC*)> callback;
	std::string text;
	TTF_Font* font;
	bool dirty = false;
};

#endif
