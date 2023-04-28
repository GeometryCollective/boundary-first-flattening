#include "bff/mesh/PolygonSoup.h"

namespace bff {

void VertexAdjacencyMaps::construct(int nV, const std::vector<int>& indices)
{
	// collect vertex-face and vertex-vertex pairs
	std::vector<std::pair<int, int>> vertexFacePairs, vertexPairs;
	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			vertexFacePairs.emplace_back(std::make_pair(i, I));
			if (i > j) std::swap(i, j);
			vertexPairs.emplace_back(std::make_pair(i, j));
		}
	}

	// sort edges and remove duplicates
	std::sort(vertexFacePairs.begin(), vertexFacePairs.end());
	std::sort(vertexPairs.begin(), vertexPairs.end());
	std::vector<std::pair<int, int>>::iterator end = std::unique(vertexPairs.begin(), vertexPairs.end());
	vertexPairs.resize(std::distance(vertexPairs.begin(), end));

	// construct maps
	eData.clear();
	fData.clear();
	eOffsets.clear();
	fOffsets.clear();

	eData.reserve(vertexPairs.size());
	fData.reserve(vertexFacePairs.size());
	eOffsets.resize(nV + 1, 0);
	fOffsets.resize(nV + 1, 0);

	if (vertexFacePairs.size() > 0) {
		int j = 0;
		for (int i = 0; i < nV; i++) {
			while (i == vertexFacePairs[j].first) {
				fData.push_back(vertexFacePairs[j].second);
				j++;
			}

			fOffsets[i + 1] = j;
		}
	}

	if (vertexPairs.size() > 0) {
		int j = 0;
		for (int i = 0; i < nV; i++) {
			while (i == vertexPairs[j].first) {
				eData.push_back(vertexPairs[j].second);
				j++;
			}

			eOffsets[i + 1] = j;
		}
	}
}

int VertexAdjacencyMaps::getAdjacentFaceCount(int v) const
{
	return fOffsets[v + 1] - fOffsets[v];
}

int VertexAdjacencyMaps::getAdjacentFaceIndex(int v, int f) const
{
	return fData[fOffsets[v] + f];
}

int VertexAdjacencyMaps::getEdgeIndex(int vi, int vj) const
{
	if (vi > vj) std::swap(vi, vj);

	int k = 0;
	for (int l = eOffsets[vi]; l < eOffsets[vi + 1]; l++) {
		if (eData[l] == vj) break;
		k++;
	}

	return eOffsets[vi] + k;
}

int VertexAdjacencyMaps::getEdgeCount() const
{
	return eOffsets[eOffsets.size() - 1];
}

void EdgeFaceAdjacencyMap::construct(const VertexAdjacencyMaps& vertexAdjacency,
									 const std::vector<int>& indices)
{
	// collect edge face pairs
	int nE = vertexAdjacency.getEdgeCount();
	std::vector<std::pair<int, int>> edgeFacePairs;
	edgeFacePairs.reserve(nE*2);

	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			int eIndex = vertexAdjacency.getEdgeIndex(i, j);
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
	data.reserve(edgeFacePairs.size());
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
