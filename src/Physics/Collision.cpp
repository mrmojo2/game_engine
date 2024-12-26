#include "Collision.h"


bool CollisionDetection::isColliding(Body* a, Body* b, CollisionInfo& ci){
	ShapeType ashapetype = a->shape->getShapeType();
	ShapeType bshapetype = b->shape->getShapeType();

	if(ashapetype == CIRCLE && bshapetype == CIRCLE){
		return isCollidingCircleCircle(a,b,ci);
	}
}

bool CollisionDetection::isCollidingCircleCircle(Body* a, Body* b, CollisionInfo& ci){
	Circle* circle_a = (Circle* ) a->shape;
	Circle* circle_b = (Circle* ) b->shape;

	Vec2 ab = b->position - a->position;
	float radius_distance = circle_a->radius + circle_b->radius;

	bool isColliding = ab.magnitudeSquared() <= radius_distance*radius_distance;
	if(!isColliding){
		return false;
	}

	//calculate collision info
	ci.a = a;
	ci.b = b;
	ci.normal = ab.unit();
	ci.contactPoint1  =  -ci.normal*circle_b->radius + b->position;
	ci.contactPoint2  =   ci.normal*circle_a->radius + a->position;
	ci.depth = (ci.contactPoint2-ci.contactPoint1).magnitude();

	return true;
}
