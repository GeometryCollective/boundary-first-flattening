#pragma once

#include <map>
#include <cmath>

typedef std::map<double, double>::iterator       KnotIter;
typedef std::map<double, double>::const_iterator KnotCIter;

class Spline {
public:
	// returns the interpolated value
	double evaluate(double t) const;

	// sets the value of the spline at a given time (i.e., knot),
	// creating a new knot at this time if necessary
	KnotIter addKnot(double t, double p);

	// removes the knot closest to the given time, within the given tolerance
	// returns true iff a knot was removed
	bool removeKnot(double t, double tolerance = 0.001);

	// sets knot entries to zero
	void reset();

	// members
	std::map<double, double> knots;
};

#include "Spline.inl"
