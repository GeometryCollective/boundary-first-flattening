#pragma once

#include <cmath>

namespace bff {

class Vector {
public:
	// initializes all components to zero
	Vector();

	// initializes with specified components
	Vector(double x, double y, double z = 0.0);

	// copy constructor
	Vector(const Vector& v);

	// access
	double& operator[](int index);
	const double& operator[](int index) const;

	// math
	Vector operator*(double s) const;
	Vector operator/(double s) const;
	Vector operator+(const Vector& v) const;
	Vector operator-(const Vector& v) const;
	Vector operator-() const;

	Vector& operator*=(double s);
	Vector& operator/=(double s);
	Vector& operator+=(const Vector& v);
	Vector& operator-=(const Vector& v);

	// returns Euclidean length
	double norm() const;

	// returns Euclidean length squared
	double norm2() const;

	// normalizes vector
	void normalize();

	// returns unit vector in the direction of this vector
	Vector unit() const;

	// members
	double x, y, z;
};

Vector operator*(double s, const Vector& v);
double dot(const Vector& u, const Vector& v);
Vector cross(const Vector& u, const Vector& v);

} // namespace bff

#include "Vector.inl"
