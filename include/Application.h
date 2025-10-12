#ifndef APPLICATION_H
#define APPLICATION

#include "Body.h"
#include "Spring.h"
#include "Vector.h"

#include <vector>
#include <SDL2/SDL.h>


class Application{
private:
	bool running = false;
	std::vector<Body*> bodies;
	Vec2 pushForce = Vec2(0,0);			//to give a force through keyboard
	int mousePosX;
	int mousePosY;	
	
	bool drawMouseImpulseLine = false;		//to give a force to a body using mouse
	int mouseImpulseBodyIndex;

	int previousFrameTime;

public:
	Application() = default;
	~Application() = default;

	bool isRunning();
	void setup();
	void input();
	void update();
	void render();
	void destroy();
};


#endif
