#include "Shape.h"
#include <limits>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//				Circle Shape Implementation
////////////////////////////////////////////////////////////////////////////////////////////////////////


Circle::Circle(float radius):radius(radius){
}

Circle::~Circle(){

}

Shape* Circle::getPointer() const{
	return new Circle(this->radius);
}

ShapeType Circle::getShapeType() const {
	return CIRCLE;
}

float Circle::getMOI(float mass) const {
	return (0.5 * mass * radius * radius);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//				Polygon Shape Implementation
////////////////////////////////////////////////////////////////////////////////////////////////////////

Polygon::Polygon(std::vector<Vec2> vertices):localVertices(vertices),worldVertices(vertices){}

Polygon::~Polygon(){}

ShapeType Polygon::getShapeType() const {
	return POLYGON;
}

float Polygon::getMOI(float mass) const {
	return 0.0;  //will implement later
}

Shape* Polygon::getPointer() const {
	return new Polygon(this->localVertices);
}

void Polygon::updateWorldVertices(float angle, const Vec2& position){
	for(auto lv: localVertices){
		Vec2 wv = lv.rotate(angle) + position;
		worldVertices.push_back(wv);
	}
}

void Polygon::clearWorldVertices(){
	worldVertices.clear();
}

Vec2 Polygon::getEdge(const int index) const {
	int next_index = (index + 1) % worldVertices.size();

	return (worldVertices[next_index] - worldVertices[index]);
}

//computes the separation between polygons a and b
	//loop all vertices of a 
	//find the normal axis
	//loop all the vertices of b
	//	project vertex of b onto normal axis
	//	keep track of min separation
	//return the best separation of all the axis
float Polygon::findMinSeparation(const Polygon* other,Vec2& contact_edge, Vec2& contact_vertex) const{
	float separation = std::numeric_limits<float>::lowest();
	
	int i=0;
	for(auto va:this->worldVertices){
		Vec2 edge = this->getEdge(i);
		Vec2 normal = edge.normal();
		float min_separation = std::numeric_limits<float>::max();
		Vec2 minVertex;
		for(auto vb:other->worldVertices){
			float projection = normal * (vb-va);
			if(projection < min_separation)	{
				min_separation = projection;
				minVertex = vb;
			};
		}
		if(min_separation > separation){
			separation = min_separation;
			contact_edge = edge;
			contact_vertex = minVertex;
		}
		i++;
	}
	return separation;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//				BOX Shape Implementation
////////////////////////////////////////////////////////////////////////////////////////////////////////

Box::Box(float width, float height): width(width),height(height){
	localVertices.push_back(Vec2( -width/2, -height/2));
	localVertices.push_back(Vec2( width/2, -height/2));
	localVertices.push_back(Vec2( width/2, height/2));
	localVertices.push_back(Vec2( -width/2, height/2));
}

Box::~Box(){}

ShapeType Box::getShapeType() const {
	return BOX;
}

float Box::getMOI(float mass) const {
	return 0.08333333333 * mass * (width*width + height*height);
}

Shape* Box::getPointer() const {
	return new Box(width, height);
}
