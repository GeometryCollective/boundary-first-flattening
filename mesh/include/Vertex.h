#pragma once

#include "Types.h"

namespace bff {

class Vertex {
public:
	// constructor
	Vertex(Mesh *mesh);

	// copy constructor
	Vertex(const Vertex& v);

	// returns one of the halfedges associated with this vertex
	HalfEdgeIter halfEdge() const;

	// sets halfedge
	void setHalfEdge(HalfEdgeCIter he);

	// returns one of the wedges (a.k.a. corner) associated with this vertex
	WedgeIter wedge() const;

	// sets mesh
	void setMesh(Mesh *mesh);

	// checks if this vertex is on the boundary
	bool onBoundary(bool checkIfOnCut = true) const;

	// checks if this vertex is isolated
	bool isIsolated() const;

	// returns degree
	int degree() const;

	// returns angle weighted average of adjacent face normals
	Vector normal() const;

	// returns 2π minus sum of incident angles. Note: only valid for interior vertices
	double angleDefect() const;

	// returns exterior angle. Note: only valid for boundary vertices
	double exteriorAngle() const;

	// position
	Vector position;

	// flag to indicate whether this vertex is a neighbor of or is
	// the north pole of a stereographic projection from the disk to a sphere
	bool inNorthPoleVicinity;

	// id between 0 and |V|-1
	int index;

	// id of the reference vertex this vertex is a duplicate of
	int referenceIndex;

private:
	// index of one of the halfedges associated with this vertex
	int halfEdgeIndex;

	// pointer to mesh this vertex belongs to
	Mesh *mesh;
};

} // namespace bff
