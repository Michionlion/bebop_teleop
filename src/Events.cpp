#include "Events.h"

std::vector<Listener*> eventListeners;

// void Listener::event(SDL_Event* e) {}

void eventPoll() {
	SDL_Event* e = NULL;
	if(SDL_PollEvent(e) && e != NULL)
		for(auto & el : eventListeners) el->event(e);
}

void registerEventListener(Listener* lis) {
	eventListeners.push_back(lis);
}

bool unregisterEventListener(Listener* lis) {
	auto beg = eventListeners.begin();
	auto end = eventListeners.end();
	for(; beg != end; ++beg) {
		if( (*beg) == lis ) {
			eventListeners.erase(beg);
			return true;
		}
	}
	return false;
}
