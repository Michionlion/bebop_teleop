#ifndef __EVENTS_H__
#define __EVENTS_H__

#include <SDL2/SDL.h>
#include <vector>

class Listener {
public:
	virtual void event(SDL_Event*) = 0;
};

extern std::vector<Listener*> eventListeners;
extern void registerEventListener(Listener*);
extern bool unregisterEventListener(Listener*);
extern void eventPoll(void);

#endif
