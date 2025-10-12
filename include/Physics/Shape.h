#ifndef SHAPE_H
#define SHAPE_H

#include "Vector.h"

#include <vector>

enum ShapeType {
	CIRCLE,
	POLYGON,
	BOX
};

class Shape{
public:	
	virtual ~Shape() = default;
	virtual ShapeType getShapeType() const = 0;
	virtual Shape*    getPointer() const = 0;
	virtual float 	  getMOI(float mass) const = 0;
	
};

class Circle:public Shape{
public:
	float radius;

	Circle(float radius);	
	~Circle();
	ShapeType getShapeType() const;
	Shape*    getPointer() const;
	float getMOI(float mass) const;
};

class Polygon:public Shape{
public:
	std::vector<Vec2> localVertices;
	std::vector<Vec2> worldVertices;

	Polygon(std::vector<Vec2> vertices);
	Polygon()=default;
	~Polygon();
	ShapeType getShapeType() const;
	Shape* getPointer() const;
	Vec2 getEdge(int index) const;
	float getMOI(float mass) const;
	void updateWorldVertices(float angle, const Vec2& position);
	void clearWorldVertices();
};

class Box:public Polygon {
public:
	float width,height;
	
	Box(float, float);
	~Box();
	ShapeType getShapeType() const;
	Shape* getPointer() const;
	float getMOI(float mass) const;
};



#endif
