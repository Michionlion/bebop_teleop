#ifndef __EVENTS_H__
#define __EVENTS_H__

#include <SDL2/SDL.h>
#include <vector>

class Listener {
public:
	Listener(void);
	virtual void event(SDL_Event*);
};

extern std::vector<Listener*> eventListeners;
extern void registerEventListener(Listener*);
extern bool unregisterEventListener(Listener*);
extern void eventPoll(void);

#endif
