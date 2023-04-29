#include "bff/mesh/PolygonSoup.h"
#include <queue>

namespace bff {

void VertexAdjacencyMaps::construct(int nV, const std::vector<int>& indices)
{
	// collect vertex-face and vertex-vertex pairs
	faceCount.clear();
	faceCount.resize(nV, std::make_pair(0, 0));
	std::vector<std::pair<int, int>> vertexPairs;
	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			faceCount[i].first++;
			faceCount[i].second = I;
			if (i < j) std::swap(i, j);
			vertexPairs.emplace_back(std::make_pair(i, j));
		}
	}

	// sort edges and remove duplicates
	std::sort(vertexPairs.begin(), vertexPairs.end());
	std::vector<std::pair<int, int>>::iterator end = std::unique(vertexPairs.begin(), vertexPairs.end());
	vertexPairs.resize(std::distance(vertexPairs.begin(), end));

	// construct maps
	data.clear();
	offsets.clear();
	data.reserve(vertexPairs.size());
	offsets.resize(nV + 1, 0);

	if (vertexPairs.size() > 0) {
		int j = 0;
		for (int i = 0; i < nV; i++) {
			while (i == vertexPairs[j].first) {
				data.emplace_back(vertexPairs[j].second);
				j++;
			}

			offsets[i + 1] = j;
		}
	}
}

std::pair<int, int> VertexAdjacencyMaps::getAdjacentFaceCount(int v) const
{
	return faceCount[v];
}

int VertexAdjacencyMaps::getEdgeIndex(int vi, int vj) const
{
	if (vi < vj) std::swap(vi, vj);

	int k = 0;
	for (int l = offsets[vi]; l < offsets[vi + 1]; l++) {
		if (data[l] == vj) break;
		k++;
	}

	return offsets[vi] + k;
}

int VertexAdjacencyMaps::getEdgeCount() const
{
	return offsets[offsets.size() - 1];
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
	std::sort(edgeFacePairs.begin(), edgeFacePairs.end());

	// construct map
	data.clear();
	data.reserve(edgeFacePairs.size());
	offsets.clear();
	offsets.resize(nE + 1, 0);

	if (edgeFacePairs.size() > 0) {
		int j = 0;
		for (int i = 0; i < nE; i++) {
			while (i == edgeFacePairs[j].first) {
				data.emplace_back(edgeFacePairs[j].second);
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

int PolygonSoup::separateFacesIntoComponents()
{
	int nComponents = 0;
	int nIndices = (int)indices.size();
	faceComponent.resize(nIndices);
	std::vector<uint8_t> seenFace(nIndices, 0);

	for (int I = 0; I < nIndices; I += 3) {
		// continue if face has already been seen
		if (seenFace[I] == 1) continue;

		// collect all faces in a single component and mark them as seen
		seenFace[I] = 1;
		faceComponent[I] = nComponents;
		std::queue<int> q;
		q.push(I);

		while (!q.empty()) {
			int f = q.front();
			q.pop();

			// loop over all edges in the face
			for (int J = 0; J < 3; J++) {
				int K = (J + 1) % 3;
				int i = indices[f + J];
				int j = indices[f + K];

				int eIndex = vertexAdjacency.getEdgeIndex(i, j);

				// check if the edge is not on the boundary
				if (edgeFaceAdjacency.getAdjacentFaceCount(eIndex) == 2) {
					int g = edgeFaceAdjacency.getAdjacentFaceIndex(eIndex, 0);
					if (g == f) g = edgeFaceAdjacency.getAdjacentFaceIndex(eIndex, 1);

					if (!seenFace[g]) {
						seenFace[g] = 1;
						faceComponent[g] = nComponents;
						q.push(g);
					}
				}
			}
		}

		nComponents++;
	}

	return nComponents;
}

} // namespace bff
