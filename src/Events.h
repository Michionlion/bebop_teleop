#include <SDL2/SDL.h>
#include <vector>

class Listener {
	virtual void event(SDL_Event*);
};

class EventDistributor {
	void poll();
};


EventDistributor eventor;
std::vector<Listener*> eventListeners;
void registerEventCallback(Listener*);
bool unregisterEventCallback(Listener*);
