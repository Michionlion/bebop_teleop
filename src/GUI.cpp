#include "GUI.h"
#include <SDL2/SDL_ttf.h>
#include <ros/ros.h>

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

void GUIC::setFG(uint8_t r, uint8_t g, uint8_t b) {
	fg.r = r;
	fg.g = g;
	fg.b = b;
}

void GUIC::setBG(uint8_t r, uint8_t g, uint8_t b) {
	bg.r = r;
	bg.g = g;
	bg.b = b;
}

void GUIC::setText(std::string str, SDL_Renderer* ren) { setText(str, ren, RESIZE_X | RESIZE_Y);}

void GUIC::setText(std::string str, SDL_Renderer* ren, short resize) {
	SDL_Surface* srf = TTF_RenderText_Shaded(font, str.c_str(), fg, bg);
	setTexture( SDL_CreateTextureFromSurface(ren, srf) );
	text = str;
	if(resize > 0) {
		SDL_SetRenderDrawColor(ren, bg.r, bg.b, bg.g, bg.a);
		SDL_RenderFillRect(ren, &bounds);
	}
	if(resize & RESIZE_X) bounds.w = -1;
	if(resize & RESIZE_Y) bounds.h = -1;


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

SDL_Texture* GUIC::getTexture() const {
	return texture;
}

void GUIC::setTexture(SDL_Texture* tex) {
	if(texture != NULL && tex != texture) {
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
	if(callback != NULL) callback(this);
}

void GUIC::setCallback(std::function<void(GUIC*)> cb) {
	callback = cb;
}
