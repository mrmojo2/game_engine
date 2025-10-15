#ifndef BODY_H
#define BODY_H

#include "Vector.h"
#include "Shape.h"

class Body{
public:
	//linear motion properties
	Vec2 position;
	Vec2 velocity     = Vec2(0.0,0.0);
	Vec2 acceleration = Vec2(0.0,0.0);
	Vec2 netForce     = Vec2(0.0,0.0);
	float mass;
	float invMass;

	//angular motion properts
	float angle = 0;
	float angular_velocity = 0;			//only 2d vector implemented so these are float for now...
	float angular_acceleration = 0;
	float netTorque = 0;
	float moi;
	float invMOI;

	float elasticity = 1;				//used to calculate the coefficient of restituion of two colliding bodies

	Shape* shape = nullptr;
	bool isColliding = false;
	bool isStatic();

	void integrateLinear(float dt);			//performs euler integration
	void integrateAngular(float dt);
	
	void addForce(const Vec2& f);
	void addTorque(const float f);
	void addImpulse(const Vec2& j);
	void addImpulse(const Vec2& j, const Vec2& r);	//to add angular impulse due to collision at r
	
	Body();
	Body(const Shape& shape,int x, int y, float mass);
	~Body();
};

#endif
