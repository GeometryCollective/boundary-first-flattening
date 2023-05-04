#pragma once

#include <stdlib.h>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "bff/linear-algebra/Vector.h"

namespace bff {

class PolygonSoup;

class VertexAdjacencyMaps {
public:
	// constructs adjacency map
	void construct(int nV, const std::vector<int>& indices);

	// returns edge index corresponding to vertex entry (vi, vj)
	int getEdgeIndex(int vi, int vj) const;

	// returns edge count
	int getEdgeCount() const;

	// clears adjacency map
	void clear();

private:
	// inserts new vertex pairs into map
	void insert(int nV, std::vector<std::pair<int, int>>& vertexPairs);

	// members
	std::vector<int> data;
	std::vector<int> offsets;
	std::vector<std::pair<int, int>> faceCount;
	friend PolygonSoup;
};

class EdgeFaceAdjacencyMap {
public:
	// constructs adjacency map
	void construct(const VertexAdjacencyMaps& vertexAdjacency,
				   const std::vector<int>& indices);

	// returns adjacent face count for edge e
	int getAdjacentFaceCount(int e) const;

	// returns face index and isAdjacent flag for edge e and 0 <= f < getAdjacentFaceCount(e)
	std::pair<int, int> getAdjacentFaceIndex(int e, int f) const;

	// clears adjacency map
	void clear();

private:
    // inserts new edge face pairs into map
	void insert(const VertexAdjacencyMaps& vertexAdjacency,
				std::vector<std::pair<int, int>>& edgeFacePairs);
			
	// members
	std::vector<int> data;
	std::vector<int> offsets;
	std::vector<uint8_t> isAdjacentFace;
	friend PolygonSoup;
};

class PolygonSoup {
public:
	// removes isolated vertices
	bool removeIsolatedVertices();

	// splits non-manifold vertices
	bool splitNonManifoldVertices();

	// orients faces and assigns components to them
	int orientFacesAndAssignComponents(std::vector<int>& faceComponent);

	// members
	std::vector<Vector> positions;
	std::vector<int> indices;
	VertexAdjacencyMaps vertexAdjacency; // build after filling positions and indices
	EdgeFaceAdjacencyMap edgeFaceAdjacency; // build after constructing vertexAdjacency

private:
	// identifies non-manifold vertices
	int identifyNonManifoldVertices(std::vector<uint8_t>& isNonManifoldVertex) const;

	// collects adjacent faces for each non-manifold vertex
	void collectAdjacentFaces(const std::vector<uint8_t>& isNonManifoldVertex,
							  std::unordered_map<int, std::vector<int>>& vertexToFacesMap) const;

	// flips face orientation if f has an edge pointing from vi to vj
	bool flipFaceOrientation(int f, int vi, int vj);
};

} // namespace bff