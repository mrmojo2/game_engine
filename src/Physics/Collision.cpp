#include "Collision.h"

#include <limits>
#include <iostream>
#include <cmath>


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

	Vec2 tangent = normal.normal();

	//calculate coefficient of restituion as harmonic mean of the elasticity property
	float e1 = a->elasticity;
	float e2 = b->elasticity;
	float e =( 2 * e1 * e2 ) / (e1 + e2);

	//calculate friction coefficient
	float f1 = a->friction;
	float f2 = b->friction;
	float f = (2*f1*f2)/(f1+f2);


	Vec2 ra = contactPoint2 - a->position;
	Vec2 rb = contactPoint1 - b->position;
	Vec2 va = a->velocity + Vec2(-a->angular_velocity*ra.y, a->angular_velocity*ra.x);		//va = vcom + (w x ra)
	Vec2 vb = b->velocity + Vec2(-b->angular_velocity*rb.y, b->angular_velocity*rb.x);

	Vec2 v_relative = va - vb;
	float v_rel_normal = v_relative * normal;							//relative velocity along the collision normal (dot product)
	float v_rel_tangential = v_relative * tangent;

	float J_normal_mag = (-(e+1)*v_rel_normal)/(a->invMass + b->invMass + pow(cross(ra,normal),2)*a->invMOI + pow(cross(rb,normal),2)*b->invMOI);
	float J_tangantial_mag = (-(f)*v_rel_tangential)/(a->invMass + b->invMass + pow(cross(ra,tangent),2)*a->invMOI + pow(cross(rb,tangent),2)*b->invMOI);

	
	Vec2 J_normal = normal * J_normal_mag;
	Vec2 J_tangential = tangent * J_tangantial_mag;
	Vec2 J_final  = J_normal + J_tangential;
	
	a->addImpulse(J_final, ra);
	b->addImpulse(-J_final, rb);
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


bool CollisionDetection::isCollidingPolygonPolygon(Body* a, Body* b, Collision& ci){
	//find the separation between a and b and b and a
	const Polygon* apoly = (Polygon*)a->shape;
	const Polygon* bpoly = (Polygon*)b->shape;
	Vec2 a_edge, b_edge;
	Vec2 a_point,b_point;
	
	if(fabs(a->angle - b->angle) < 1e-10){		//TODO:collision resolution doesnt work for perfectly aligned polygons so for now just change the angle of one of them a little bit. fix resolution code later
		a->angle += 0.0001;	
	}
	
	float ab_separation = apoly->findMinSeparation(bpoly,a_edge,b_point);
	if(ab_separation>=0){
		return false;
	}
	
	float ba_separation = bpoly->findMinSeparation(apoly,b_edge,a_point);
	if(ba_separation>=0){
		return false;
	}
	
	/*if(fabs(a->angle - b->angle) < 1e-10){		
		a->angle -= 0.0001;	
	}*/

	ci.a = a;
	ci.b = b;
	if(ab_separation > ba_separation){
		ci.normal = a_edge.normal();
		ci.depth = -1.0 * ab_separation;
		ci.contactPoint1 = b_point;
		ci.contactPoint2 = b_point + ci.normal * ci.depth;
	}else if(ba_separation > ab_separation){
		ci.normal = -b_edge.normal();				//because in resolvePenetration() it is assumed that the collision normal is always from a to b
		ci.depth = -1.0 * ba_separation;
		ci.contactPoint2 = a_point;
		ci.contactPoint1 = a_point - ci.normal * ci.depth;		//because normal is going from a to b to find contanct point of b we must take normal go
	}
	return true;
}
