#pragma once

#include "bff/mesh/Types.h"

namespace bff {

class Face {
public:
	// constructor
	Face(Mesh *mesh);

	// copy constructor
	Face(const Face& f);

	// one of the halfedges associated with this face
	HalfEdgeIter halfEdge() const;

	// sets halfedge
	void setHalfEdge(HalfEdgeCIter he);

	// sets mesh
	void setMesh(Mesh *mesh);

	// checks if this face is real
	bool isReal() const;

	// flag to indicate whether this face fills a hole
	bool fillsHole;

	// flag to indicate whether this face is incident to the north
	// pole of a stereographic projection from the disk to a sphere
	bool inNorthPoleVicinity;

	// id between 0 and |F|-1
	int index;

private:
	// index of one of the halfedges associated with this face
	int halfEdgeIndex;

	// pointer to mesh this face belongs to
	Mesh *mesh;
};

} // namespace bff
