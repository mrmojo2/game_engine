#ifndef COLLISION_H
#define COLLISION_H


#include "Body.h"

////////////////////////////////////////////////////////////////////////////////////////
//	class to handle collision of two bodies
////////////////////////////////////////////////////////////////////////////////////////

class Collision{
public:
	//information of collision of two bodies
	Body* a;
	Body* b;
	Vec2 normal;
	Vec2 contactPoint1;
	Vec2 contactPoint2;	
	float depth;
	
	
	void resolvePenetration();
	void resolveCollision();
};

//////////////////////////////////////////////////////////////////////////////////////////
//	class to handle collision detection between objects
/////////////////////////////////////////////////////////////////////////////////////////

class CollisionDetection{
public:
	static bool isColliding(Body* a, Body* b, Collision& ci);
	static bool isCollidingCircleCircle(Body* a, Body* b, Collision& ci);
};



#endif
