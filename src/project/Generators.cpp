#include "bff/project/Generators.h"
#include <queue>
#include <unordered_set>

namespace bff {

void Generators::buildPrimalSpanningTree(Mesh& mesh, VertexData<VertexCIter>& primalParent)
{
	// mark each vertex as its own parent
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		primalParent[v] = v;
	}

	// build primal spanning tree
	VertexCIter root = mesh.vertices.begin();
	std::queue<VertexCIter> q;
	q.push(root);

	while (!q.empty()) {
		VertexCIter u = q.front();
		q.pop();

		HalfEdgeCIter he = u->halfEdge();
		do {
			HalfEdgeCIter flip = he->flip();
			EdgeCIter e = he->edge();

			if (e->isCuttable) {
				VertexCIter v = flip->vertex();

				if (primalParent[v] == v && v != root) {
					primalParent[v] = u;
					q.push(v);
				}
			}

			he = flip->next();
		} while (he != u->halfEdge());
	}
}

bool Generators::inPrimalSpanningTree(EdgeCIter e, const VertexData<VertexCIter>& primalParent)
{
	HalfEdgeCIter he = e->halfEdge();
	VertexCIter u = he->vertex();
	VertexCIter v = he->flip()->vertex();

	return primalParent[u] == v || primalParent[v] == u;
}

void Generators::buildDualSpanningTree(Mesh& mesh,
									   FaceData<FaceCIter>& ngon,
									   FaceData<FaceCIter>& dualParent,
									   const VertexData<VertexCIter>& primalParent)
{
	int uncuttableEdges = 0;
	FaceData<std::vector<HalfEdgeCIter>> ngonHalfEdges(mesh);

	// mark each ngon face as its own parent and collect halfedges
	for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
		if (uncuttableEdges > 0) {
			uncuttableEdges--;
			continue;
		}

		HalfEdgeCIter he = f->halfEdge();
		while (!he->edge()->isCuttable) he = he->next();
		HalfEdgeCIter fhe = he;
		std::unordered_set<int> seenUncuttableEdges;

		do {
			ngon[he->face()] = f;
			ngonHalfEdges[f].emplace_back(he);

			he = he->next();
			while (!he->edge()->isCuttable) {
				seenUncuttableEdges.emplace(he->edge()->index);
				he = he->flip()->next();
			}

		} while (he != fhe);

		uncuttableEdges = (int)seenUncuttableEdges.size();
		if (f->fillsHole) uncuttableEdges--;
		dualParent[f] = f;
	}

	// build dual spanning tree
	FaceCIter root = mesh.faces.begin();
	std::queue<FaceCIter> q;
	q.push(root);

	while (!q.empty()) {
		FaceCIter f = q.front();
		q.pop();

		const std::vector<HalfEdgeCIter>& halfEdges = ngonHalfEdges[f];
		for (int i = 0; i < (int)halfEdges.size(); i++) {
			HalfEdgeCIter he = halfEdges[i];
			EdgeCIter e = he->edge();

			if (!inPrimalSpanningTree(e, primalParent)) {
				FaceCIter g = ngon[he->flip()->face()];

				if (dualParent[g] == g && g != root) {
					dualParent[g] = f;
					q.push(g);
				}
			}
		}
	}
}

bool Generators::inDualSpanningTree(EdgeCIter e,
									const FaceData<FaceCIter>& ngon,
									const FaceData<FaceCIter>& dualParent)
{
	HalfEdgeCIter he = e->halfEdge();
	FaceCIter f = ngon[he->face()];
	FaceCIter g = ngon[he->flip()->face()];

	return dualParent[f] == g || dualParent[g] == f;
}

EdgeIter Generators::sharedEdge(VertexCIter u, VertexCIter v)
{
	HalfEdgeCIter he = u->halfEdge();

	do {
		if (he->flip()->vertex() == v) {
			return he->edge();
		}

		he = he->flip()->next();
	} while (he != u->halfEdge());

	std::cerr << "Code should not reach here!" << std::endl;
	return he->edge();
}

void Generators::createBoundary(Mesh& mesh)
{
	int nVertices = (int)mesh.vertices.size();
	int nEdges = (int)mesh.edges.size();
	int nHalfEdges1 = (int)mesh.halfEdges.size();
	int nV = 0;
	int nE = 0;
	int nHe = 0;

	// insert vertices, edges and halfedges
	VertexData<uint8_t> seenVertex(mesh, 0);
	EdgeData<uint8_t> seenEdge(mesh, 0);
	for (WedgeIter w: mesh.cutBoundary()) {
		VertexIter v = w->vertex();
		HalfEdgeIter he = w->halfEdge()->next();
		EdgeIter e = he->edge();

		// insert vertex
		if (!seenVertex[v]) {
			seenVertex[v] = 1;

		} else {
			int index = v->index;
			const Vector& position = v->position;
			v = mesh.vertices.emplace(mesh.vertices.end(), Vertex(&mesh));
			v->position = position;
			v->index = nVertices + nV++;
			v->referenceIndex = index;
		}

		//  insert edge and halfedge
		if (!seenEdge[e]) {
			seenEdge[e] = 1;

		} else {
			e = mesh.edges.emplace(mesh.edges.end(), Edge(&mesh));
			e->index = nEdges + nE++;
		}

		HalfEdgeIter newHe = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
		newHe->index = nHalfEdges1 + nHe++;
		newHe->setVertex(v);
		newHe->setEdge(e);
		newHe->setFlip(he);
	}

	// insert boundary face
	FaceIter newF = mesh.boundaries.emplace(mesh.boundaries.end(), Face(&mesh));
	newF->setHalfEdge(mesh.halfEdges.begin() + nHalfEdges1);
	newF->index = 0;

	// update connectivity
	int nHalfEdges2 = (int)mesh.halfEdges.size();
	for (int j = nHalfEdges1; j < nHalfEdges2; j++) {
		int i = j == nHalfEdges1 ? nHalfEdges2 - 1 : j - 1;
		int k = j == nHalfEdges2 - 1 ? nHalfEdges1 : j + 1;

		HalfEdgeIter newPrev = mesh.halfEdges.begin() + i;
		HalfEdgeIter newHe = mesh.halfEdges.begin() + j;
		HalfEdgeIter newNext = mesh.halfEdges.begin() + k;
		HalfEdgeIter flip = newHe->flip();

		// update vertex connectivity
		VertexIter v = newHe->vertex();
		v->setHalfEdge(newHe);
		HalfEdgeIter h = flip->next();
		do {
			h->setVertex(v);

			if (h->edge()->onCut) break;
			h = h->flip()->next();
		} while (true);

		// update edge connectivity
		EdgeIter e = newHe->edge();
		e->setHalfEdge(flip);
		e->onGenerator = true;

		// update face connectivity
		newHe->setFace(newF);

		// update halfedge connectivity
		flip->setFlip(newHe);
		newHe->setPrev(newPrev);
		newHe->setNext(newNext);
		newHe->onBoundary = true;
	}

	// assign halfedge edges and remove cut label from all edges
	for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		e->halfEdge()->setEdge(e);
		e->onCut = false;
	}
}

void Generators::compute(Mesh& mesh)
{
	// build primal and dual spanning trees
	VertexData<VertexCIter> primalParent(mesh);
	FaceData<FaceCIter> dualParent(mesh), ngon(mesh);
	buildPrimalSpanningTree(mesh, primalParent);
	buildDualSpanningTree(mesh, ngon, dualParent, primalParent);

	// label generators
	for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		if (!inPrimalSpanningTree(e, primalParent) &&
			!inDualSpanningTree(e, ngon, dualParent) &&
			e->isCuttable) {
			HalfEdgeCIter he = e->halfEdge();

			// track vertices back to the root
			std::vector<EdgeIter> temp1;
			VertexCIter u = he->vertex();
			while (primalParent[u] != u) {
				VertexCIter v = primalParent[u];
				temp1.emplace_back(sharedEdge(u, v));
				u = v;
			}

			std::vector<EdgeIter> temp2;
			u = he->flip()->vertex();
			while (primalParent[u] != u) {
				VertexCIter v = primalParent[u];
				temp2.emplace_back(sharedEdge(u, v));
				u = v;
			}

			// temporarily label edges as cuts
			e->onCut = true;
			for (int i = 0; i < temp1.size(); i++) temp1[i]->onCut = true;
			for (int i = 0; i < temp2.size(); i++) temp2[i]->onCut = true;
		}
	}

	// create boundary from generators
	createBoundary(mesh);
}

} // namespace bff
