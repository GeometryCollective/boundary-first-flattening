#pragma once

#include "bff/mesh/Types.h"

namespace bff {

class HalfEdge {
public:
	// constructor
	HalfEdge(Mesh *mesh);

	// copy constructor
	HalfEdge(const HalfEdge& he);

	// returns the next halfedge (in CCW order) associated with this halfedge's face
	HalfEdgeIter next() const;

	// returns the prev halfedge associated with this halfedge's face
	HalfEdgeIter prev() const;

	// returns the other halfedge associated with this halfedge's edge
	HalfEdgeIter flip() const;

	// returns the vertex at the base of this halfedge
	VertexIter vertex() const;

	// returns the edge associated with this halfedge
	EdgeIter edge() const;

	// returns the face associated with this halfedge
	FaceIter face() const;

	// returns the corner opposite to this halfedge. Undefined if this halfedge
	// is on the boundary
	CornerIter corner() const;

	// returns the wedge (a.k.a. corner) associated with this halfedge
	WedgeIter wedge() const;

	// sets next halfedge
	void setNext(HalfEdgeCIter he);

	// sets prev halfedge
	void setPrev(HalfEdgeCIter he);

	// sets flip halfedge
	void setFlip(HalfEdgeCIter he);

	// sets vertex
	void setVertex(VertexCIter v);

	// sets edge
	void setEdge(EdgeCIter e);

	// sets face
	void setFace(FaceCIter f);

	// sets corner
	void setCorner(CornerCIter c);

	// sets mesh
	void setMesh(Mesh *mesh);

	// boolean flag to indicate if halfedge is on the boundary
	bool onBoundary;

	// id between 0 and |H|-1
	int index;

private:
	// indices of adjacent mesh elements
	int nextIndex;
	int prevIndex;
	int flipIndex;
	int vertexIndex;
	int edgeIndex;
	int faceIndex;
	int cornerIndex;

	// pointer to mesh this halfedge belongs to
	Mesh *mesh;
};

} // namespace bff
