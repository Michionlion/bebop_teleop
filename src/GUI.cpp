#include "GUI.h"
#include <SDL2/SDL_ttf.h>
#include <ros/ros.h>

SDL_Color fg = {255, 255, 255, 255};
SDL_Color bg = {  0, 0, 0, 255};

GUIC::GUIC(TTF_Font* f, int x, int y, int width, int height) {
	font = f;
	bounds.x = x;
	bounds.y = y;
	bounds.w = width;
	bounds.h = height;
}

GUIC::~GUIC() {
	destroy();
}

void GUIC::destroy() {
	if(texture != NULL) SDL_DestroyTexture(texture);
	font = NULL;
	texture = NULL;
}

void GUIC::setText(std::string str, SDL_Renderer* ren) {
	SDL_Surface* srf = TTF_RenderText_Shaded(font, str.c_str(), fg, bg);
	setTexture( SDL_CreateTextureFromSurface(ren, srf) );
	text = str;

	// ROS_ERROR( "TEXT: %s", str.c_str() );
	TTF_SizeText(font, str.c_str(), bounds.w < 0 ? &bounds.w : NULL, bounds.h < 0 ? &bounds.h : NULL);

	SDL_FreeSurface(srf);
}

SDL_Rect* GUIC::getBounds() {
	return &bounds;
}

std::string GUIC::getText() const {
	return text;
}

void GUIC::setTexture(SDL_Texture* tex) {
	if(texture != NULL) {
		SDL_DestroyTexture(texture);
		texture = NULL;
	}
	texture = tex;
	dirty = true;
}

void GUIC::render(SDL_Renderer* ren) {
	if(dirty && texture != NULL && ren != NULL) {
		// ROS_INFO("REND");
		SDL_RenderCopy(ren, texture, NULL, &bounds);
		dirty = false;
	}
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
