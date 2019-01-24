#pragma once

#include "Types.h"

namespace bff {

class Face {
public:
	// constructor
	Face();

	// one of the halfedges associated with this face
	HalfEdgeIter he;

	// flag to indicate whether this face fills a hole
	bool fillsHole;

	// flag to indicate whether this face is incident to the north
	// pole of a stereographic projection from the disk to a sphere
	bool inNorthPoleVicinity;

	// id between 0 and |F|-1
	int index;

	// returns face normal
	Vector normal(bool normalize = true) const;

	// returns face area
	double area() const;

	// checks if this face is real
	bool isReal() const;
};

} // namespace bff

#include "Face.inl"
