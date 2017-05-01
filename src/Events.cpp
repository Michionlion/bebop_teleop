#include "Events.h"
#include <ros/ros.h>

std::vector<Listener*> eventListeners;

// void Listener::event(SDL_Event* e) {}

void publishEvent(SDL_Event* event) {
	for(auto & el : eventListeners) el->event(event);
}

void eventPoll() {
	SDL_Event e;
	while( SDL_PollEvent(&e) )
		for(auto & el : eventListeners) el->event(&e);
}

void registerEventListener(Listener* lis) {
	// ROS_INFO("REGISTERED");
	eventListeners.push_back(lis);

	// ROS_INFO( "SIZE: %ld", eventListeners.size() );
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
