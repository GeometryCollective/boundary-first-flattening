#include "bff/mesh/PolygonSoup.h"

namespace bff {

void VertexEdgeAdjacencyMap::construct(int nV, const std::vector<int>& indices)
{
	// collect vertex pairs
	std::vector<std::pair<int, int>> vertexPairs;
	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			if (i > j) std::swap(i, j);
			vertexPairs.emplace_back(std::make_pair(i, j));
		}
	}

	// sort edges and remove duplicates
	std::sort(vertexPairs.begin(), vertexPairs.end());
	std::vector<std::pair<int, int>>::iterator end = std::unique(vertexPairs.begin(), vertexPairs.end());
	vertexPairs.resize(std::distance(vertexPairs.begin(), end));

	// construct map
	data.clear();
	offsets.clear();
	offsets.resize(nV + 1, 0);

	if (vertexPairs.size() > 0) {
		int j = 0;
		for (int i = 0; i < nV; i++) {
			while (i == vertexPairs[j].first) {
				data.push_back(vertexPairs[j].second);
				j++;
			}

			offsets[i + 1] = j;
		}
	}
}

int VertexEdgeAdjacencyMap::getEdgeIndex(int vi, int vj) const
{
	if (vi > vj) std::swap(vi, vj);

	int k = 0;
	for (int l = offsets[vi]; l < offsets[vi + 1]; l++) {
		if (data[l] == vj) break;
		k++;
	}

	return offsets[vi] + k;
}

int VertexEdgeAdjacencyMap::getEdgeCount() const
{
	return offsets[offsets.size() - 1];
}

void EdgeFaceAdjacencyMap::construct(const VertexEdgeAdjacencyMap& vertexEdgeAdjacency,
									 const std::vector<int>& indices)
{
	// collect edge face pairs
	int nE = vertexEdgeAdjacency.getEdgeCount();
	std::vector<std::pair<int, int>> edgeFacePairs;
	edgeFacePairs.reserve(nE*2);

	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			int eIndex = vertexEdgeAdjacency.getEdgeIndex(i, j);
			edgeFacePairs.emplace_back(std::make_pair(eIndex, I));
		}
	}

	// sort edge face pairs
	std::sort(edgeFacePairs.begin(), edgeFacePairs.end(), 
		[](const std::pair<int,int>& a, const std::pair<int,int>& b) {
		return a.first < b.first;
	});

	// construct map
	data.clear();
	offsets.clear();
	offsets.resize(nE + 1, 0);

	if (edgeFacePairs.size() > 0) {
		int j = 0;
		for (int i = 0; i < nE; i++) {
			while (i == edgeFacePairs[j].first) {
				data.push_back(edgeFacePairs[j].second);
				j++;
			}

			offsets[i + 1] = j;
		}
	}
}

int EdgeFaceAdjacencyMap::getAdjacentFaceCount(int e) const
{
	return offsets[e + 1] - offsets[e];
}

int EdgeFaceAdjacencyMap::getAdjacentFaceIndex(int e, int f) const
{
	return data[offsets[e] + f];
}

} // namespace bff
