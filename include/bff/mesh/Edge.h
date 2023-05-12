#pragma once

#include "bff/mesh/Types.h"

namespace bff {

class Edge {
public:
	// constructor
	Edge(Mesh *mesh);

	// copy constructor
	Edge(const Edge& e);

	// returns one of the halfedges associated with this edge
	HalfEdgeIter halfEdge() const;

	// sets halfedge
	void setHalfEdge(HalfEdgeCIter he);

	// sets mesh
	void setMesh(Mesh *mesh);

	// checks if this edge is on the boundary
	bool onBoundary() const;

	// boolean flag to indicate if edge is on a generator
	bool onGenerator;

	// boolean flag to indicate if edge is on a cut
	bool onCut;

	// boolean flag to indicate if cut can pass through edge
	bool isCuttable;

	// id between 0 and |E|-1
	int index;

private:
	// index of one of the halfedges associated with this edge
	int halfEdgeIndex;

	// pointer to mesh this edge belongs to
	Mesh *mesh;
};

} // namespace bff
