#pragma once

#include <set>
#include <fstream>
#include <sstream>
#include "MeshData.h"

namespace bff {

class AdjacencyTable {
public:
	// constructs table
	void construct(int n, const std::vector<int>& indices);

	// returns unique index corresponding to entry (i, j)
	int getIndex(int i, int j) const;

	// returns table size
	int getSize() const;

private:
	// members
	std::vector<std::set<int>> data;
	std::vector<int> iMap;
	int size;
};

class PolygonSoup {
public:
	std::vector<Vector> positions;
	std::vector<int> indices;
	AdjacencyTable table; // construct after filling positions and indices
};

class MeshIO {
public:
	// reads model from obj file
	static bool read(const std::string& fileName, Model& model, std::string& error);

	// writes data to obj file
	static bool write(const std::string& fileName, Model& model,
					  const std::vector<bool>& mappedToSphere, bool normalize);

	// separates model into components
	static void separateComponents(const PolygonSoup& soup,
								   const std::vector<int>& isCuttableEdge,
								   std::vector<PolygonSoup>& soups,
								   std::vector<std::vector<int>>& isCuttableEdgeSoups,
								   std::vector<std::pair<int, int>>& modelToMeshMap,
								   std::vector<std::vector<int>>& meshToModelMap);

	// builds a halfedge mesh
	static bool buildMesh(const PolygonSoup& soup,
						  const std::vector<int>& isCuttableEdge,
						  Mesh& mesh, std::string& error);

	// centers model around origin and records radius
	static void normalize(Model& model);

private:
	// preallocates mesh elements
	static void preallocateElements(const PolygonSoup& soup, Mesh& mesh);

	// checks if mesh has isolated vertices
	static bool hasIsolatedVertices(const Mesh& mesh);

	// checks if mesh has non-manifold vertices
	static bool hasNonManifoldVertices(const Mesh& mesh);

	// reads data from obj file
	static bool read(std::istringstream& in, Model& model, std::string& error);

	// writes data to obj file
	static void write(std::ofstream& out, Model& model,
					  const std::vector<bool>& mappedToSphere, bool normalize);
};

} // namespace bff
