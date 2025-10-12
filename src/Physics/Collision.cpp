#include "Collision.h"

#include <limits>


void Collision::resolvePenetration(){
	if(a->isStatic() && b->isStatic()) return;
	//if two bodies intersect then reposition them such that they are just touching by taking into account their mass

	float da = depth * a->invMass / (a->invMass + b->invMass) ;		//this is the same as da = depth * mb/ma+mb
	float db = depth * b->invMass / (a->invMass + b->invMass) ;		//but using invrse mass takes care of the case where the other objecct is static )

	a->position += -normal*da;
	b->position +=  normal*db;
}

void Collision::resolveCollision(){
	resolvePenetration();

	//calculate coefficient of restituion as harmonic mean of the bounciness property
	float e1 = a->bounciness;
	float e2 = b->bounciness;
	float e =( 2 * e1 * e2 ) / (e1 + e2);

	//calculate the impulse
	float v_separating = (a->velocity - b->velocity) * normal;			//it is the speed of one object relative to another, in the direction between the two objects
	float J_mag = (-(e+1) * v_separating )/(a->invMass + b->invMass); 		//think about divide by 0 later (static objects dont really collide)

	//apply the  impuse
	a->addImpulse(normal * J_mag);
	b->addImpulse(-normal * J_mag);
}



bool CollisionDetection::isColliding(Body* a, Body* b, Collision& ci){
	ShapeType ashapetype = a->shape->getShapeType();
	ShapeType bshapetype = b->shape->getShapeType();

	if(ashapetype == CIRCLE && bshapetype == CIRCLE){
		return isCollidingCircleCircle(a,b,ci);
	}else if((ashapetype == POLYGON || ashapetype == BOX) && (bshapetype == POLYGON || bshapetype == BOX)){
		return isCollidingPolygonPolygon(a,b,ci);	
	}
}

bool CollisionDetection::isCollidingCircleCircle(Body* a, Body* b, Collision& ci){
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

//computes the separation between polygons a and b
float findMinSeparation(const Polygon* a, const Polygon* b){
	float separation = std::numeric_limits<float>::lowest();

	//loop all vertices of a 
	//find the normal axis
	//loop all the vertices of b
	//	project vertex of b onto normal axis
	//	keep track of min separation
	//return the best separation of all the axis
	
	int i=0;
	for(auto va:a->worldVertices){
		Vec2 normal = a->getEdge(i).normal();
		float min_separation = std::numeric_limits<float>::max();
		for(auto vb:b->worldVertices){
			float projection = normal * (vb-va);
			if(projection < min_separation)	min_separation = projection;
		}
		if(min_separation > separation){
			separation = min_separation;
		}
		i++;
	}
	return separation;
}

bool CollisionDetection::isCollidingPolygonPolygon(Body* a, Body* b, Collision& ci){
	//find the separation between a and b and b and a
	const Polygon* apoly = (Polygon*)a->shape;
	const Polygon* bpoly = (Polygon*)b->shape;

	if(findMinSeparation(apoly,bpoly)<=0 && findMinSeparation(bpoly,apoly)<=0){
		return true;
	}
	return false;
}
