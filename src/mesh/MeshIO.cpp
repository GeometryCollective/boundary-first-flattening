#include "bff/mesh/MeshIO.h"
#include "bff/project/BinPacking.h"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <iomanip>

namespace bff {

void AdjacencyTable::construct(int n, const std::vector<int>& indices)
{
	data.clear();
	iMap.clear();
	data.resize(n);
	iMap.resize(n);
	size = 0;

	// build table
	for (int I = 0; I < (int)indices.size(); I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = indices[I + J];
			int j = indices[I + K];

			if (i > j) std::swap(i, j);
			data[i].emplace(j);
		}
	}

	// build iMap
	for (int i = 0; i < n; i++) {
		iMap[i] = size;
		size += data[i].size();
	}
}

int AdjacencyTable::getIndex(int i, int j) const
{
	if (i > j) std::swap(i, j);

	int k = 0;
	for (std::set<int>::iterator it = data[i].begin(); it != data[i].end(); it++) {
		if (*it == j) break;
		k++;
	}

	return iMap[i] + k;
}

int AdjacencyTable::getSize() const
{
	return size;
}

bool MeshIO::readOBJ(const std::string& fileName, PolygonSoup& soup,
					 std::vector<std::pair<int, int>>& uncuttableEdges,
					 std::string& error)
{
	std::ifstream in(fileName.c_str());
	std::istringstream buffer;

	if (!in.is_open()) {
		error = "Unable to open file";
		return false;

	} else {
		buffer.str(std::string(std::istreambuf_iterator<char>(in),
							   std::istreambuf_iterator<char>()));
	}

	std::string line;
	int nVertices = 0;
	bool seenFace = false;
	soup.positions.clear();
	soup.indices.clear();
	uncuttableEdges.clear();

	while (std::getline(buffer, line)) {
		std::istringstream ss(line);
		std::string token;
		ss >> token;

		if (token == "v") {
			double x, y, z;
			ss >> x >> y >> z;
			soup.positions.emplace_back(Vector(x, y, z));

			if (seenFace) {
				nVertices = 0;
				seenFace = false;
			}
			nVertices++;

		} else if (token == "f") {
			seenFace = true;
			int vertexCount = 0;
			int rootIndex = 0;
			int prevIndex = 0;

			while (ss >> token) {
				std::istringstream indexStream(token);
				std::string indexString;

				if (std::getline(indexStream, indexString, '/')) {
					int index = std::stoi(indexString);
					if (index < 0) index = nVertices + index;
					else index -= 1;

					if (vertexCount == 0) rootIndex = index;

					vertexCount++;
					if (vertexCount > 3) {
						// triangulate polygon if vertexCount > 3
						soup.indices.emplace_back(rootIndex);
						soup.indices.emplace_back(prevIndex);
						soup.indices.emplace_back(index);

						// tag edge as uncuttable
						int i = rootIndex;
						int j = prevIndex;
						std::pair<int, int> edge(i, j);
						uncuttableEdges.emplace_back(edge);

					} else {
						soup.indices.emplace_back(index);
					}

					prevIndex = index;
				}
			}
		}
	}

	in.close();
	return true;
}

void MeshIO::separateComponents(const PolygonSoup& soup,
								const std::vector<uint8_t>& isCuttableModelEdge,
								std::vector<PolygonSoup>& soups,
								std::vector<std::vector<uint8_t>>& isCuttableSoupEdge,
								std::vector<std::pair<int, int>>& modelToMeshMap,
								std::vector<std::vector<int>>& meshToModelMap)
{
	// create an edge face adjacency map
	int nIndices = (int)soup.indices.size();
	int nVertices = (int)soup.positions.size();
	std::vector<std::vector<int>> adjacentFaces(soup.table.getSize());
	for (int I = 0; I < nIndices; I += 3) {
		for (int J = 0; J < 3; J++) {
			int K = (J + 1) % 3;
			int i = soup.indices[I + J];
			int j = soup.indices[I + K];

			int eIndex = soup.table.getIndex(i, j);
			adjacentFaces[eIndex].emplace_back(I);

			// check for non-manifold edges
			if (adjacentFaces[eIndex].size() > 2) {
				soups.emplace_back(soup);
				isCuttableSoupEdge.emplace_back(isCuttableModelEdge);
				return;
			}
		}
	}

	// separate soup into components
	int components = 0;
	std::vector<uint8_t> seenFace(nIndices, 0);
	std::vector<int> faceComponent(nIndices);
	for (int I = 0; I < nIndices; I += 3) {
		// continue if face has already been seen
		if (seenFace[I] == 1) continue;

		// collect all faces in a single component and mark them as seen
		seenFace[I] = 1;
		faceComponent[I] = components;
		std::queue<int> q;
		q.push(I);

		while (!q.empty()) {
			int f = q.front();
			q.pop();

			// loop over all edges in the face
			for (int J = 0; J < 3; J++) {
				int K = (J + 1) % 3;
				int i = soup.indices[f + J];
				int j = soup.indices[f + K];

				int eIndex = soup.table.getIndex(i, j);

				// check if the edge is not on the boundary
				const std::vector<int>& faces = adjacentFaces[eIndex];
				if (faces.size() == 2) {
					int g = f == faces[0] ? faces[1] : faces[0];

					if (!seenFace[g]) {
						seenFace[g] = 1;
						faceComponent[g] = components;
						q.push(g);
					}
				}
			}
		}

		components++;
	}

	meshToModelMap.resize(components);
	if (components == 1) {
		soups.emplace_back(soup);
		isCuttableSoupEdge.emplace_back(isCuttableModelEdge);

		for (int i = 0; i < nVertices; i++) {
			meshToModelMap[0].emplace_back(i);
			modelToMeshMap.emplace_back(std::make_pair(0, i));
		}

	} else {
		// create soups
		soups.resize(components);
		isCuttableSoupEdge.resize(components);
		std::vector<std::unordered_map<int, int>> soupVertexIndexMap(components);
		modelToMeshMap.resize(nVertices);

		for (int I = 0; I < nIndices; I += 3) {
			int c = faceComponent[I];

			for (int J = 0; J < 3; J++) {
				int i = soup.indices[I + J];

				// insert vertex if it hasn't been seen
				if (soupVertexIndexMap[c].find(i) == soupVertexIndexMap[c].end()) {
					int index = (int)soups[c].positions.size();
					soupVertexIndexMap[c][i] = index;
					soups[c].positions.emplace_back(soup.positions[i]);
					meshToModelMap[c].emplace_back(i);
					modelToMeshMap[i] = std::make_pair(c, index);
				}

				// add index
				soups[c].indices.emplace_back(soupVertexIndexMap[c][i]);
			}
		}

		// construct tables
		for (int c = 0; c < components; c++) {
			soups[c].table.construct(soups[c].positions.size(), soups[c].indices);
			isCuttableSoupEdge[c].resize(soups[c].table.getSize(), 1);
		}

		// mark cuttable edges for each soup
		for (int I = 0; I < nIndices; I += 3) {
			int c = faceComponent[I];

			for (int J = 0; J < 3; J++) {
				int K = (J + 1) % 3;
				int i = soup.indices[I + J];
				int j = soup.indices[I + K];

				int eIndex = soup.table.getIndex(i, j);

				// add edge if uncuttable
				if (!isCuttableModelEdge[eIndex]) {
					int ii = soupVertexIndexMap[c][i];
					int jj = soupVertexIndexMap[c][j];

					int eIndexSoup = soups[c].table.getIndex(ii, jj);
					isCuttableSoupEdge[c][eIndexSoup] = 0;
				}
			}
		}
	}
}

void MeshIO::preallocateElements(const PolygonSoup& soup, Mesh& mesh)
{
	// clear arrays
	mesh.vertices.clear();
	mesh.edges.clear();
	mesh.faces.clear();
	mesh.corners.clear();
	mesh.halfEdges.clear();
	mesh.boundaries.clear();

	// reserve space (reserving extra for hole filling and generators)
	int nVertices = (int)soup.positions.size();
	int nEdges = soup.table.getSize();
	int nFaces = (int)soup.indices.size()/3;
	int nCorners = 3*nFaces;
	int nHalfedges = 2*nEdges;

	mesh.vertices.reserve(2*nVertices);
	mesh.edges.reserve(2*nEdges);
	mesh.faces.reserve(2*nFaces);
	mesh.corners.reserve(2*nCorners);
	mesh.halfEdges.reserve(2*nHalfedges);
}

bool MeshIO::hasIsolatedVertices(const Mesh& mesh)
{
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		if (v->isIsolated()) {
			return true;
		}
	}

	return false;
}

bool MeshIO::hasNonManifoldVertices(const Mesh& mesh)
{
	VertexData<int> adjacentFaces(mesh, 0);
	for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
		HalfEdgeCIter h = f->halfEdge();
		do {
			adjacentFaces[h->vertex()]++;

			h = h->next();
		} while (h != f->halfEdge());
	}

	for (BoundaryCIter b = mesh.boundaries.begin(); b != mesh.boundaries.end(); b++) {
		HalfEdgeCIter h = b->halfEdge();
		do {
			adjacentFaces[h->vertex()]++;

			h = h->next();
		} while (h != b->halfEdge());
	}

	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		if (adjacentFaces[v] != v->degree()) {
			return true;
		}
	}

	return false;
}

bool MeshIO::buildMesh(const PolygonSoup& soup,
					   const std::vector<uint8_t>& isCuttableEdge,
					   Mesh& mesh, std::string& error)
{
	// preallocate elements
	preallocateElements(soup, mesh);

	// create and insert vertices
	int nVertices = (int)soup.positions.size();
	std::vector<VertexIter> indexToVertex(nVertices);
	for (int i = 0; i < nVertices; i++) {
		VertexIter v = mesh.vertices.emplace(mesh.vertices.end(), Vertex(&mesh));
		v->position = soup.positions[i];
		v->index = (int)mesh.vertices.size() - 1;
		indexToVertex[i] = v;
	}

	// create and insert halfedges, edges and "real" faces
	int tableSize = soup.table.getSize();
	std::vector<int> edgeCount(tableSize, 0);
	std::vector<HalfEdgeIter> existingHalfEdges(tableSize);
	std::vector<int> hasFlipEdge(2*tableSize, 0);

	for (int I = 0; I < (int)soup.indices.size(); I += 3) {
		// create new face
		FaceIter f = mesh.faces.emplace(mesh.faces.end(), Face(&mesh));
		f->index = (int)mesh.faces.size() - 1;

		// create a halfedge for each edge of the newly created face
		std::vector<HalfEdgeIter> halfEdges(3);
		for (int J = 0; J < 3; J++) {
			halfEdges[J] = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
			halfEdges[J]->index = (int)mesh.halfEdges.size() - 1;
		}

		// initialize the newly created halfedges
		for (int J = 0; J < 3; J++) {
			// current halfedge goes from vertex i to vertex j
			int K = (J + 1) % 3;
			int i = soup.indices[I + J];
			int j = soup.indices[I + K];

			// set the current halfedge's attributes
			HalfEdgeIter h = halfEdges[J];
			h->setNext(halfEdges[K]);
			h->setPrev(halfEdges[(J + 3 - 1) % 3]);
			h->onBoundary = false;
			hasFlipEdge[h->index] = 0;

			// point the new halfedge and vertex i to each other
			VertexIter v = indexToVertex[i];
			h->setVertex(v);
			v->setHalfEdge(h);

			// point the new halfedge and face to each other
			h->setFace(f);
			f->setHalfEdge(h);

			int eIndex = soup.table.getIndex(i, j);
			if (edgeCount[eIndex] > 0) {
				// if a halfedge between vertex i and j has been created in the past,
				// then it is the flip halfedge of the current halfedge
				HalfEdgeIter flip = existingHalfEdges[eIndex];
				h->setFlip(flip);
				flip->setFlip(h);
				h->setEdge(flip->edge());

				hasFlipEdge[h->index] = 1;
				hasFlipEdge[flip->index] = 1;
				edgeCount[eIndex]++;

			} else {
				// create an edge and set its halfedge
				EdgeIter e = mesh.edges.emplace(mesh.edges.end(), Edge(&mesh));
				e->index = (int)mesh.edges.size() - 1;
				h->setEdge(e);
				e->setHalfEdge(h);
				e->isCuttable = isCuttableEdge[eIndex];

				// record the newly created edge and halfedge from vertex i to j
				existingHalfEdges[eIndex] = h;
				edgeCount[eIndex] = 1;
			}

			// check for non-manifold edges
			if (edgeCount[eIndex] > 2) {
				mesh.status = Mesh::ErrorCode::nonManifoldEdges;
				error = "Mesh has non-manifold edges.";
				return false;
			}
		}
	}

	// create and insert boundary halfedges and "imaginary" faces for boundary cycles
	// also create and insert corners
	for (HalfEdgeIter h = mesh.halfEdges.begin(); h != mesh.halfEdges.end(); h++) {
		// if a halfedge has no flip halfedge, create a new face and
		// link it the corresponding boundary cycle
		if (!hasFlipEdge[h->index]) {
			// create new face
			FaceIter f = mesh.boundaries.emplace(mesh.boundaries.end(), Face(&mesh));
			f->index = (int)mesh.boundaries.size() - 1;

			// walk along boundary cycle
			std::vector<HalfEdgeIter> boundaryCycle;
			boundaryCycle.reserve(2*mesh.edges.size() - mesh.halfEdges.size());
			HalfEdgeIter he = h;
			do {
				// create a new halfedge
				HalfEdgeIter bH = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
				bH->index = (int)mesh.halfEdges.size() - 1;
				boundaryCycle.emplace_back(bH);

				// grab the next halfedge along the boundary that does not have a
				// flip halfedge
				HalfEdgeIter nextHe = he->next();
				while (hasFlipEdge[nextHe->index]) {
					nextHe = nextHe->flip()->next();
				}

				// set the current halfedge's attributes
				bH->setVertex(nextHe->vertex());
				bH->setEdge(he->edge());
				bH->onBoundary = true;

				// point the new halfedge and face to each other
				bH->setFace(f);
				f->setHalfEdge(bH);

				// point the new halfedge and he to each other
				bH->setFlip(he);
				he->setFlip(bH);

				// continue walk
				he = nextHe;
			} while (he != h);

			// link the cycle of boundary halfedges together
			int n = (int)boundaryCycle.size();
			for (int i = 0; i < n; i++) {
				boundaryCycle[i]->setNext(boundaryCycle[(i + n - 1) % n]); // boundary halfedges are linked in clockwise order
				boundaryCycle[i]->setPrev(boundaryCycle[(i + 1) % n]);
				hasFlipEdge[boundaryCycle[i]->index] = 1;
				hasFlipEdge[boundaryCycle[i]->flip()->index] = 1;
			}
		}

		// point the newly created corner and its halfedge to each other
		if (!h->onBoundary) {
			CornerIter c = mesh.corners.emplace(mesh.corners.end(), Corner(&mesh));
			c->index = (int)mesh.corners.size() - 1;
			c->setHalfEdge(h);
			h->setCorner(c);
		}
	}

	// check if mesh has isolated vertices
	if (hasIsolatedVertices(mesh)) {
		mesh.status = Mesh::ErrorCode::isolatedVertices;
		error = "Mesh has isolated vertices.";
		return false;
	}

	// check if mesh has non-manifold vertices
	if (hasNonManifoldVertices(mesh)) {
		mesh.status = Mesh::ErrorCode::nonManifoldVertices;
		error = "Mesh has non manifold vertices.";
		return false;
	}

	return true;
}

void MeshIO::normalize(Model& model)
{
	// compute center of mass
	Vector cm;
	int nVertices = 0;
	for (int i = 0; i < model.size(); i++) {
		for (VertexCIter v = model[i].vertices.begin(); v != model[i].vertices.end(); v++) {
			cm += v->position;
			nVertices++;
		}
	}
	cm /= nVertices;

	// translate to origin and determine radius
	double radius = 1e-8;
	for (int i = 0; i < model.size(); i++) {
		for (VertexIter v = model[i].vertices.begin(); v != model[i].vertices.end(); v++) {
			v->position -= cm;
			radius = std::max(radius, v->position.norm());
		}
	}

	// rescale to unit radius
	for (int i = 0; i < model.size(); i++) {
		for (VertexIter v = model[i].vertices.begin(); v != model[i].vertices.end(); v++) {
			v->position /= radius;
		}

		model[i].radius = radius;
		model[i].cm = cm;
	}
}

bool MeshIO::buildModel(const std::vector<std::pair<int, int>>& uncuttableEdges,
						PolygonSoup& soup, Model& model, std::string& error)
{
	// construct adjacency table
	soup.table.construct(soup.positions.size(), soup.indices);
	std::vector<uint8_t> isCuttableModelEdge(soup.table.getSize(), 1);
	for (int i = 0; i < (int)uncuttableEdges.size(); i++) {
		int eIndex = soup.table.getIndex(uncuttableEdges[i].first, uncuttableEdges[i].second);
		isCuttableModelEdge[eIndex] = 0;
	}

	// separate model into components
	std::vector<PolygonSoup> soups;
	std::vector<std::vector<uint8_t>> isCuttableSoupEdge;
	separateComponents(soup, isCuttableModelEdge, soups, isCuttableSoupEdge,
					   model.modelToMeshMap, model.meshToModelMap);

	// build halfedge meshes
	model.meshes.resize(soups.size());
	for (int i = 0; i < (int)soups.size(); i++) {
		if (!buildMesh(soups[i], isCuttableSoupEdge[i], model[i], error)) {
			return false;
		}
	}

	// normalize model
	normalize(model);
	return true;
}

bool MeshIO::read(const std::string& fileName, Model& model, std::string& error)
{
	// read polygon soup from obj file
	PolygonSoup soup;
	std::vector<std::pair<int, int>> uncuttableEdges;
	if (!readOBJ(fileName, soup, uncuttableEdges, error)) {
		return false;
	}

	// build model
	if (!buildModel(uncuttableEdges, soup, model, error)) {
		return false;
	}

	return true;
}

void MeshIO::packUvs(Model& model, const std::vector<uint8_t>& isSurfaceMappedToSphere,
					 std::vector<Vector>& originalUvIslandCenters,
					 std::vector<Vector>& newUvIslandCenters,
					 std::vector<uint8_t>& isUvIslandFlipped,
					 Vector& modelMinBounds, Vector& modelMaxBounds)
{
	if (model.size() > 1) {
		for (int i = 0; i < model.size(); i++) {
			model[i].projectUvsToPcaAxis();
		}
	}

	BinPacking::pack(model, isSurfaceMappedToSphere, originalUvIslandCenters,
					 newUvIslandCenters, isUvIslandFlipped,
					 modelMinBounds, modelMaxBounds);
}

void MeshIO::collectModelUvs(Model& model, bool normalizeUvs,
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
							 std::vector<int>& indicesOffset)
{
	// initialize
	int nVertices = model.nVertices();
	int nFaces = 0;
	for (int i = 0; i < model.size(); i++) {
		nFaces += (int)model[i].faces.size();
	}

	positions.clear(); positions.reserve(nVertices);
	uvs.clear(); uvs.reserve(nVertices);
	vIndices.clear(); vIndices.reserve(3*nFaces);
	uvIndices.clear(); uvIndices.reserve(3*nFaces);
	indicesOffset.clear(); indicesOffset.reserve(nFaces + 1);
	indicesOffset.emplace_back(0);

	// collect vertex positions
	for (int i = 0; i < nVertices; i++) {
		std::pair<int, int> vData = model.localVertexIndex(i);
		const Mesh& mesh = model[vData.first];
		VertexCIter v = mesh.vertices.begin() + vData.second;

		if (isSurfaceMappedToSphere[vData.first] == 1) {
			const Vector& uv = v->wedge()->uv;
			positions.emplace_back(uv);

		} else {
			Vector p = v->position*mesh.radius + mesh.cm;
			positions.emplace_back(p);
		}
	}

	// collect UVs and indices
	int nV = 0;
	int nUvs = 0;
	for (int i = 0; i < model.size(); i++) {
		// compute uv radius and shift
		Vector minExtent(modelMinBounds.x, modelMinBounds.y);
		double dx = modelMaxBounds.x - minExtent.x;
		double dy = modelMaxBounds.y - minExtent.y;
		double extent = std::max(dx, dy);
		minExtent.x -= (extent - dx)/2.0;
		minExtent.y -= (extent - dy)/2.0;

		// compute sphere radius if component has been mapped to a sphere
		double sphereRadius = 1.0;
		if (isSurfaceMappedToSphere[i] == 1) {
			for (WedgeCIter w = model[i].wedges().begin(); w != model[i].wedges().end(); w++) {
				sphereRadius = std::max(w->uv.norm(), sphereRadius);
			}
		}

		// collect interior UVs
		int uvCount = 0;
		HalfEdgeData<int> uvIndexMap(model[i]);

		for (VertexCIter v = model[i].vertices.begin(); v != model[i].vertices.end(); v++) {
			if (!v->onBoundary()) {
				Vector uv = v->wedge()->uv;
				if (isSurfaceMappedToSphere[i] == 1) {
					uv /= sphereRadius;
					uv.x = 0.5 + atan2(uv.z, uv.x)/(2*M_PI);
					uv.y = 0.5 - asin(uv.y)/M_PI;

				} else {
					uv *= model[i].radius;
				}

				uv -= originalUvIslandCenters[i];
				if (isUvIslandFlipped[i] == 1) uv = Vector(-uv.y, uv.x);
				uv += newUvIslandCenters[i];
				uv -= minExtent;
				if (normalizeUvs) uv /= extent;
				uvs.emplace_back(uv);

				HalfEdgeCIter he = v->halfEdge();
				do {
					uvIndexMap[he->next()] = uvCount;

					he = he->flip()->next();
				} while (he != v->halfEdge());

				uvCount++;
			}
		}

		// collect boundary UVs
		for (WedgeCIter w: model[i].cutBoundary()) {
			Vector uv = w->uv;
			if (isSurfaceMappedToSphere[i] == 1) {
				uv /= sphereRadius;
				uv.x = 0.5 + atan2(uv.z, uv.x)/(2*M_PI);
				uv.y = 0.5 - asin(uv.y)/M_PI;

			} else {
				uv *= model[i].radius;
			}

			uv -= originalUvIslandCenters[i];
			if (isUvIslandFlipped[i] == 1) uv = Vector(-uv.y, uv.x);
			uv += newUvIslandCenters[i];
			uv -= minExtent;
			if (normalizeUvs) uv /= extent;
			uvs.emplace_back(uv);

			HalfEdgeCIter he = w->halfEdge()->prev();
			do {
				uvIndexMap[he->next()] = uvCount;

				if (he->edge()->onCut) break;
				he = he->flip()->next();
			} while (!he->onBoundary);

			uvCount++;
		}

		// collect indices
		int uncuttableEdges = 0;
		for (FaceCIter f = model[i].faces.begin(); f != model[i].faces.end(); f++) {
			if (!f->fillsHole) {
				if (uncuttableEdges > 0) {
					uncuttableEdges--;
					continue;
				}

				HalfEdgeCIter he = f->halfEdge()->next();
				while (!he->edge()->isCuttable) he = he->next();
				HalfEdgeCIter fhe = he;
				std::unordered_set<int> seenUncuttableEdges;

				do {
					VertexCIter v = he->vertex();
					int vIndex = v->referenceIndex == -1 ? v->index : v->referenceIndex;
					vIndices.emplace_back(model.globalVertexIndex(i, vIndex));
					uvIndices.emplace_back(nUvs + uvIndexMap[he->next()]);
					nV++;

					he = he->next();
					while (!he->edge()->isCuttable) {
						seenUncuttableEdges.emplace(he->edge()->index);
						he = he->flip()->next();
					}

				} while (he != fhe);

				indicesOffset.emplace_back(nV);
				uncuttableEdges = (int)seenUncuttableEdges.size();
			}
		}

		nUvs += uvCount;
	}
}

void writeString(std::ofstream& out, const std::string& str)
{
	out.write(str.c_str(), str.size());
}

bool MeshIO::writeOBJ(const std::string& fileName, bool writeOnlyUvs,
					  const std::vector<Vector>& positions,
					  const std::vector<Vector>& uvs,
					  const std::vector<int>& vIndices,
					  const std::vector<int>& uvIndices,
					  const std::vector<int>& indicesOffset)
{
	std::ofstream out(fileName.c_str());
	if (!out.is_open()) {
		return false;
	}

	if (!writeOnlyUvs) {
		for (int i = 0; i < (int)positions.size(); i++) {
			const Vector& p = positions[i];
			writeString(out, "v " + std::to_string(p.x) + " " +
									std::to_string(p.y) + " " +
									std::to_string(p.z) + "\n");
		}
	}

	for (int i = 0; i < (int)uvs.size(); i++) {
		const Vector& uv = uvs[i];
		if (writeOnlyUvs) {
			writeString(out, "v " + std::to_string(uv.x) + " " +
									std::to_string(uv.y) + " 0.0\n");

		} else {
			writeString(out, "vt " + std::to_string(uv.x) + " " +
									 std::to_string(uv.y) + "\n");
		}
	}

	for (int i = 0; i < (int)indicesOffset.size() - 1; i++) {
		writeString(out, "f");

		for (int j = indicesOffset[i]; j < indicesOffset[i + 1]; j++) {
			if (writeOnlyUvs) {
				writeString(out, " " + std::to_string(uvIndices[j] + 1));

			} else {
				writeString(out, " " + std::to_string(vIndices[j] + 1) + "/" +
									   std::to_string(uvIndices[j] + 1));
			}
		}

		writeString(out, "\n");
	}

	out.close();
	return true;
}

bool MeshIO::write(const std::string& fileName, Model& model,
				   const std::vector<uint8_t>& isSurfaceMappedToSphere,
				   bool normalizeUvs, bool writeOnlyUvs)
{
	// pack UVs
	std::vector<Vector> originalUvIslandCenters, newUvIslandCenters;
	std::vector<uint8_t> isUvIslandFlipped;
	Vector modelMinBounds, modelMaxBounds;
	packUvs(model, isSurfaceMappedToSphere, originalUvIslandCenters,
			newUvIslandCenters, isUvIslandFlipped,
			modelMinBounds, modelMaxBounds);

	// collect model UVs
	std::vector<Vector> positions, uvs;
	std::vector<int> vIndices, uvIndices, indicesOffset;
	collectModelUvs(model, normalizeUvs, isSurfaceMappedToSphere,
					originalUvIslandCenters, newUvIslandCenters,
					isUvIslandFlipped, modelMinBounds, modelMaxBounds,
					positions, uvs, vIndices, uvIndices, indicesOffset);

	// write OBJ
	return writeOBJ(fileName, writeOnlyUvs, positions, uvs,
					vIndices, uvIndices, indicesOffset);
}

} // namespace bff
