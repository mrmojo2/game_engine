#include "Application.h"
#include "Graphics.h"
#include "Constants.h"
#include "Force.h"
#include "Collision.h"
#include "Utils.h"

#include <iostream>
#include <stdint.h>

bool Application::isRunning(){
	return running;
}

void Application::setup(){
	running = Graphics::OpenWindow();
	previousFrameTime = SDL_GetTicks();
	

	SDL_GetMouseState(&mousePosX, &mousePosY);

	int ww = Graphics::windowWidth;
	int wh = Graphics::windowHeight;
	

	Body *bottom = new Body(Box(ww-10,100),ww/2,wh - 75,0.0);
	bottom->elasticity = 0.2;
	bodies.push_back(bottom);
	
	/*Body *right = new Body(Box(100,wh-10),ww-75,wh/2,0.0);
	right->elasticity = 0.2;
	bodies.push_back(right);
	
	Body *left = new Body(Box(100,wh-10),75,wh/2,0.0);
	left->elasticity = 0.2;
	bodies.push_back(left);*/
	
	Body *b2 = new Body(Box(200,50),ww/2,wh/2+100,0.0);
	b2->elasticity = 0.2;
	b2->angle =  0.5;
	bodies.push_back(b2);
 

	/*Body *b3 = new Body(Box(200,200),0,0,1.0);
	b3->elasticity = 0.3;
	bodies.push_back(b3);*/
}

void Application::input(){
	static int buttonDownTime, buttonUpTime;
	SDL_Event event;
	while(SDL_PollEvent(&event)){
		switch (event.type){
			case SDL_QUIT:
				running = false;
				break;
			case SDL_KEYDOWN:
				if(event.key.keysym.sym == SDLK_ESCAPE)
					running = false;
				if(event.key.keysym.sym == SDLK_UP)
					pushForce.y = -100*PIXELS_PER_METER;
				if(event.key.keysym.sym == SDLK_DOWN)
					pushForce.y = 100*PIXELS_PER_METER;
				if(event.key.keysym.sym == SDLK_LEFT)
					pushForce.x = -100*PIXELS_PER_METER;
				if(event.key.keysym.sym == SDLK_RIGHT)
					pushForce.x = +100*PIXELS_PER_METER;
				break;
			case SDL_KEYUP:
				if(event.key.keysym.sym == SDLK_UP)
					pushForce.y = 0;
				if(event.key.keysym.sym == SDLK_DOWN)
					pushForce.y = 0;
				if(event.key.keysym.sym == SDLK_LEFT)
					pushForce.x = 0;
				if(event.key.keysym.sym == SDLK_RIGHT)
					pushForce.x = 0;
				break;
			case SDL_MOUSEMOTION:
				mousePosX = event.motion.x;
				mousePosY = event.motion.y;

				/*bodies[4]->position.x = mousePosX;
				bodies[4]->position.y = mousePosY;*/

				break;
			case SDL_MOUSEBUTTONDOWN:
				if(event.button.button == SDL_BUTTON_LEFT){
					Body *box = new Body(Box(50,50),mousePosX, mousePosY, 1.0);
					//box->angle = Utils::get_random_float(0.0,3.1415);
					box->elasticity  = 0.4;
					bodies.push_back(box);
				}
				break;

		}
	}
}
void Application::update(){
	Graphics::ClearScreen(0xFF112233);
	//maintain constant fps
	//static int  previousFrameTime; 
	int delay = MILLISECONDS_PER_FRAME - (SDL_GetTicks()-previousFrameTime);
	if(delay>0){				//delay can be negative if a frame loop take more time than MILLISECONDS_PER_FRAME
		SDL_Delay(delay);
	}
	
	//delta time: difference between the current frame and the last frame in seconds
	//used to implement fps independant movement
	float deltaTime = (SDL_GetTicks() - previousFrameTime)/1000.0f;
	if(deltaTime > MILLISECONDS_PER_FRAME/1000.0f){
		deltaTime = MILLISECONDS_PER_FRAME/1000.0f;
	}	

	//set time of this frame to be used in the next frame
	previousFrameTime = SDL_GetTicks();

	
	//apply forces to the bodies
	for(auto body:bodies){
		//weight force
		body->addForce(Vec2(0.0,9.8*PIXELS_PER_METER*body->mass));
		
		//pushForce from keyboard
		body->addForce(pushForce);

		
		//drag force
		Vec2 drag = Force::getDragForce(*body, 0.002);
		//body->addForce(drag);
		
		//apply torque
		//body->addTorque(200);
	}


	//perform integration, transformation and rotation
	for(auto body: bodies){
		body->integrateLinear(deltaTime);
		body->integrateAngular(deltaTime);

		bool isPolygon = body->shape->getShapeType() == POLYGON ||  body->shape->getShapeType() == BOX;
		if(isPolygon){
			Polygon* p = (Polygon* ) body->shape;
			p->updateWorldVertices(body->angle, body->position);
		}
	}


	//perform collision detection between bodies
	if(bodies.size()>=2){	
	for(int i=0; i < bodies.size()-1 ; i++){
		for(int j = i+1 ; j < bodies.size() ; j++){
			Body* a = bodies[i];
			Body* b = bodies[j];
			
			a->isColliding = false;
			b->isColliding = false;

			Collision collision;
			if(CollisionDetection::isColliding(a,b,collision)){
				a->isColliding = true;
				b->isColliding = true;

				Graphics::DrawFillCircle(collision.contactPoint1.x, collision.contactPoint1.y,5,0xffff0000);
				Graphics::DrawFillCircle(collision.contactPoint2.x, collision.contactPoint2.y,5,0xff0000ff);
	
				collision.resolveCollision();
			}
		}
	}
	}


	//window boundary
	for(auto body:bodies){
		if(body->shape->getShapeType() == CIRCLE){
			Circle* c = (Circle* ) body->shape;
			if((body->position.y > (Graphics::windowHeight - c->radius))){
				body->position.y = Graphics::windowHeight - c->radius;	//putting body on the edges if it exceds
				body->velocity.y *= -0.9;						//making the collision not perfectly elastic
			}else if(body->position.y < (0 + c->radius)){
				body->position.y = c->radius;
				body->velocity.y *= -0.9;
			}
			if(body->position.x > (Graphics::windowWidth - c->radius)){
				body->position.x = Graphics::windowWidth - c->radius;	//putting body on the edges if it exceds
				body->velocity.x *= -0.9;						//making the collision not perfectly elastic
			}else if(body->position.x < (0 + c->radius)){
				body->position.x = c->radius;
				body->velocity.x *= -0.9;
			}	
		}
	}
	
}
void Application::render(){
	
	if(drawMouseImpulseLine){
		//Graphics::DrawLine(bodies[mouseImpulseBodyIndex]->position.x, bodies[mouseImpulseBodyIndex]->position.y, , mousePos.y, 0xff0000ff);
	}

	//render the bodies
	for(auto body:bodies){
		Uint32 color = body->isColliding ? 0xff0000ff: 0xffffffff; 
		if(body->shape->getShapeType() == CIRCLE){
			Circle* c = (Circle* )body->shape;
			Graphics::DrawCircle(body->position.x,body->position.y,c->radius,body->angle,color);
		}else if(body->shape->getShapeType() == BOX){
			Box* b = (Box* )body->shape;
			Graphics::DrawPolygon(body->position.x, body->position.y, b->worldVertices,color);
			b->clearWorldVertices();
		}
	}
	Graphics::RenderFrame();	
}
void Application::destroy(){
	for(auto body:bodies){
		delete body;
	}
	Graphics::CloseWindow();
}
