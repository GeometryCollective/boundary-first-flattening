#pragma once

#include <fstream>
#include <sstream>
#include "bff/mesh/MeshData.h"

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
	std::vector<int> data;
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
	// reads polygon soup from file and builds model
	static bool read(const std::string& fileName, Model& model, std::string& error);

	// reads polygon soup from obj file
	static bool readOBJ(const std::string& fileName, PolygonSoup& soup,
						std::vector<std::pair<int, int>>& uncuttableEdges,
						std::string& error);

	// builds model
	static bool buildModel(const std::vector<std::pair<int, int>>& uncuttableEdges,
						   PolygonSoup& soup, Model& model, std::string& error);

	// writes model and its UVs to file
	static bool write(const std::string& fileName, Model& model,
					  const std::vector<uint8_t>& isSurfaceMappedToSphere,
					  bool normalizeUvs, bool writeOnlyUvs);

	// bin packs model UV islands
	static void packUvs(Model& model, const std::vector<uint8_t>& isSurfaceMappedToSphere,
						std::vector<Vector>& originalUvIslandCenters,
						std::vector<Vector>& newUvIslandCenters,
						std::vector<uint8_t>& isUvIslandFlipped,
						Vector& modelMinBounds, Vector& modelMaxBounds);

	// collects model positions and UVs
	static void collectModelUvs(Model& model, bool normalizeUvs,
								const std::vector<uint8_t>& isSurfaceMappedToSphere,
								const std::vector<Vector>& originalUvIslandCenters,
								const std::vector<Vector>& newUvIslandCenters,
								const std::vector<uint8_t>& isUvIslandFlipped,
								const Vector& modelMinBounds,
								const Vector& modelMaxBounds,
								std::vector<Vector>& positions,
								std::vector<Vector>& uvs,
								std::vector<int>& vIndices,
								std::vector<int>& uvIndices,
								std::vector<int>& indicesOffset);

	// writes model positions and UVs to obj file
	static bool writeOBJ(const std::string& fileName, bool writeOnlyUvs,
						 const std::vector<Vector>& positions,
						 const std::vector<Vector>& uvs,
						 const std::vector<int>& vIndices,
						 const std::vector<int>& uvIndices,
						 const std::vector<int>& indicesOffset);

private:
	// separates model into components
	static void separateComponents(const PolygonSoup& soup,
								   const std::vector<uint8_t>& isCuttableModelEdge,
								   std::vector<PolygonSoup>& soups,
								   std::vector<std::vector<uint8_t>>& isCuttableSoupEdge,
								   std::vector<std::pair<int, int>>& modelToMeshMap,
								   std::vector<std::vector<int>>& meshToModelMap);

	// preallocates mesh elements
	static void preallocateElements(const PolygonSoup& soup, Mesh& mesh);

	// checks if mesh has isolated vertices
	static bool hasIsolatedVertices(const Mesh& mesh);

	// checks if mesh has non-manifold vertices
	static bool hasNonManifoldVertices(const Mesh& mesh);

	// builds a halfedge mesh
	static bool buildMesh(const PolygonSoup& soup,
						  const std::vector<uint8_t>& isCuttableEdge,
						  Mesh& mesh, std::string& error);

	// centers model around origin and records radius
	static void normalize(Model& model);
};

} // namespace bff
