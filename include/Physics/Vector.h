#ifndef VECTOR_H
#define VECTOR_H

class Vec2{
public:
	float x,y;
	
	//construcors & destructors
	Vec2();
	Vec2(float x, float y);
	Vec2(const Vec2& v);
	~Vec2() = default;
	
	//methods
	float magnitude() const;
	float magnitudeSquared() const;
	
	Vec2 unit() const;
	Vec2 normal() const;
	Vec2 rotate(const float angle) const;
	Vec2& normalize();
	Vec2& scale(const float n);

	Vec2 operator  + (const Vec2& v) const;
	Vec2 operator  - (const Vec2& v) const;
	Vec2 operator  * (const float s) const;
	Vec2 operator  / (const float s) const;
	Vec2 operator  - ();
	float operator * (const Vec2& v) const;

	bool operator  == (const Vec2& v) const;
	bool operator  != (const Vec2& v) const;

	Vec2& operator += (const Vec2& v);
	Vec2& operator -= (const Vec2& v);
	
	friend float cross(const Vec2& v1, const Vec2& v2);		//cross of two 2d vector will only have the z component so storing just the magnitude of that z component. can use the vector as (0,0,mag) if needed
};


#endif
