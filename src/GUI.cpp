#include "GUI.h"
#include <SDL2/SDL_ttf.h>

GUIC::GUIC(int x, int y, int width, int height) {
	bounds.x = x;
	bounds.y = y;
	bounds.w = width;
	bounds.h = height;
}

GUIC::~GUIC() {
	SDL_DestroyTexture(texture);
}

void GUIC::setText(std::string str) {
	text = str;
}

std::string GUIC::getText() const {
	return text;
}

void GUIC::setTexture(SDL_Texture* tex) {
	texture = tex;
}

void GUIC::render(SDL_Renderer* ren) const {
	SDL_RenderCopy(ren, texture, NULL, &bounds);
}

bool GUIC::inside(int x, int y) const {
	return x > bounds.x && y > bounds.y && x < bounds.x + bounds.w && y < bounds.y + bounds.h;
}

void GUIC::callCB() {
	if(callback != NULL) callback->call(this);
}

void GUIC::setCallback(Callback* cb) {
	callback = cb;
}
