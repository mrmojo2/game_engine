#include "Application.h"
#include "Graphics.h"
#include "Constants.h"
#include "Force.h"
#include "Collision.h"

#include <iostream>
#include <stdint.h>

bool Application::isRunning(){
	return running;
}

void Application::setup(){
	running = Graphics::OpenWindow();
	previousFrameTime = SDL_GetTicks();
	

	SDL_GetMouseState(&mousePosX, &mousePosY);
	

	Body *b1 = new Body(Box(100.0,100.0),Graphics::windowWidth/2,Graphics::windowHeight/2,1.0);
	Body *b2 = new Body(Box(100.0,100.0),Graphics::windowWidth/2+120,Graphics::windowHeight/2,1.0);
	b1->angular_velocity = 0.5;
	b2->angular_velocity = 0.3;
	bodies.push_back(b1);	
	bodies.push_back(b2);	
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
				break;
			case SDL_MOUSEBUTTONDOWN:
				if(event.button.button == SDL_BUTTON_LEFT){
				}
				break;

			/*case SDL_MOUSEBUTTONDOWN:{
				int curMouseX = event.motion.x;
				int curMouseY = event.motion.y;		
				if(event.button.button == SDL_BUTTON_LEFT){
					for(int i=0;i<bodies.size();i++){
						if( (curMouseX > bodies[i]->position.x - (bodies[i]->radius+5)) && (curMouseX < bodies[i]->position.x + (bodies[i]->radius+5)) && 
						    (curMouseY > bodies[i]->position.y - (bodies[i]->radius+5)) && (curMouseY < bodies[i]->position.y + (bodies[i]->radius+5)) ){
							drawMouseImpulseLine = true;
							mouseImpulseBodyIndex = i;
						}
					}
					buttonDownTime = SDL_GetTicks();
				}
				break;
			}
			
			case SDL_MOUSEBUTTONUP:
				if(event.button.button == SDL_BUTTON_LEFT){
					buttonUpTime = SDL_GetTicks();
					if(drawMouseImpulseLine){
						drawMouseImpulseLine = false;
						Vec2 impulseDir = (bodies[mouseImpulseBodyIndex]->position - mousePos).unit();
						float impulseMag = (bodies[mouseImpulseBodyIndex]->position - mousePos).magnitude();
						bodies[mouseImpulseBodyIndex]->velocity = impulseDir*impulseMag;
					}else{
						//Body* p = new Body(event.motion.x, event.motion.y, (buttonUpTime-buttonDownTime)/50);
						//bodies.push_back(p);
					}
				}
				break;*/
				


		}
	}
}
void Application::update(){
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
		//body->addForce(Vec2(0.0,9.8*PIXELS_PER_METER*body->mass));
		
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
	Graphics::ClearScreen(0xFF112233);
	
	if(drawMouseImpulseLine){
		//Graphics::DrawLine(bodies[mouseImpulseBodyIndex]->position.x, bodies[mouseImpulseBodyIndex]->position.y, , mousePos.y, 0xff0000ff);
	}

	//render the bodies
	for(auto body:bodies){
		Uint32 color = body->isColliding ? 0xff0000ff: 0xffffffff; 
		if(body->shape->getShapeType() == CIRCLE){
			Circle* c = (Circle* )body->shape;
			Graphics::DrawFillCircle(body->position.x,body->position.y,c->radius,color);
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
