#pragma once

#include "HalfEdge.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "Corner.h"
#include "CutIterator.h"

namespace bff {

class Mesh {
public:
	// constructor
	Mesh();

	// copy constructor
	Mesh(const Mesh& mesh);

	// returns euler characteristic
	int eulerCharacteristic() const;

	// returns mesh diameter
	double diameter() const;

	// range based for loop over the cut boundary
	CutPtrSet cutBoundary(); // valid only for 1 boundary loop, by default, this is the first loop

	// returns reference to wedges (a.k.a. corners)
	std::vector<Wedge>& wedges();
	const std::vector<Wedge>& wedges() const;

	// error (and warning) codes
	enum class ErrorCode {
		ok,
		nonManifoldEdges,
		nonManifoldVertices,
		isolatedVertices
	};

	// member variables
	std::vector<Vertex> vertices;
	std::vector<Edge> edges;
	std::vector<Face> faces;
	std::vector<Corner> corners;
	std::vector<HalfEdge> halfEdges;
	std::vector<Face> boundaries;
	double radius;
	Vector cm;
	ErrorCode status;
};

} // namespace bff
