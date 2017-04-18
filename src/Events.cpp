#include "Events.h"
#include <algorithm>

void EventDistributor::poll() {
	SDL_Event* e = NULL;
	if(SDL_PollEvent(e) && e != NULL) {
		auto beg = eventListeners.begin();
		auto end = eventListeners.end();
		for(; beg != end; ++beg) {
			(*beg)(e);
		}
	}
}

void registerEventCallback(EventListener lis) {
	eventListeners.push_back(lis);
}

bool unregisterEventCallback(EventListener lis) {
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
