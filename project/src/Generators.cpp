#include "Generators.h"
#include <queue>
#include <unordered_map>

namespace bff {

void Generators::buildPrimalSpanningTree(Mesh& mesh,
										 VertexData<VertexCIter>& primalParent)
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

		HalfEdgeCIter he = u->he;
		do {
			HalfEdgeCIter flip = he->flip;
			EdgeCIter e = he->edge;

			if (e->isCuttable) {
				VertexCIter v = flip->vertex;

				if (primalParent[v] == v && v != root) {
					primalParent[v] = u;
					q.push(v);
				}
			}

			he = flip->next;
		} while (he != u->he);
	}
}

bool Generators::inPrimalSpanningTree(EdgeCIter e,
									  const VertexData<VertexCIter>& primalParent)
{
	HalfEdgeCIter he = e->he;
	VertexCIter u = he->vertex;
	VertexCIter v = he->flip->vertex;

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

		HalfEdgeCIter he = f->he;
		while (!he->edge->isCuttable) he = he->next;
		HalfEdgeCIter fhe = he;
		std::unordered_map<int, bool> seenUncuttableEdges;

		do {
			ngon[he->face] = f;
			ngonHalfEdges[f].push_back(he);

			he = he->next;
			while (!he->edge->isCuttable) {
				seenUncuttableEdges[he->edge->index] = true;
				he = he->flip->next;
			}

		} while (he != fhe);

		uncuttableEdges = (int)seenUncuttableEdges.size();
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
			EdgeCIter e = he->edge;

			if (!inPrimalSpanningTree(e, primalParent)) {
				FaceCIter g = ngon[he->flip->face];

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
	HalfEdgeCIter he = e->he;
	FaceCIter f = ngon[he->face];
	FaceCIter g = ngon[he->flip->face];

	return dualParent[f] == g || dualParent[g] == f;
}

EdgeIter Generators::sharedEdge(VertexCIter u, VertexCIter v)
{
	HalfEdgeCIter he = u->he;

	do {
		if (he->flip->vertex == v) {
			return he->edge;
		}

		he = he->flip->next;
	} while (he != u->he);

	std::cerr << "Code should not reach here!" << std::endl;
	return he->edge;
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
	VertexData<int> seenVertex(mesh, 0);
	EdgeData<int> seenEdge(mesh, 0);
	for (WedgeIter w: mesh.cutBoundary()) {
		VertexIter v = w->vertex();
		HalfEdgeIter he = w->he->next;
		EdgeIter e = he->edge;

		// insert vertex
		if (!seenVertex[v]) {
			seenVertex[v] = 1;

		} else {
			int index = v->index;
			const Vector& position = v->position;
			v = mesh.vertices.insert(mesh.vertices.end(), Vertex());
			v->position = position;
			v->index = nVertices + nV++;
			v->referenceIndex = index;
		}

		//  insert edge and halfedge
		if (!seenEdge[e]) {
			seenEdge[e] = 1;

		} else {
			e = mesh.edges.insert(mesh.edges.end(), Edge());
			e->index = nEdges + nE++;
		}

		HalfEdgeIter newHe = mesh.halfEdges.insert(mesh.halfEdges.end(), HalfEdge());
		newHe->index = nHalfEdges1 + nHe++;
		newHe->vertex = v;
		newHe->edge = e;
		newHe->flip = he;
	}

	// insert boundary face
	FaceIter newF = mesh.boundaries.insert(mesh.boundaries.end(), Face());
	newF->he = mesh.halfEdges.begin() + nHalfEdges1;
	newF->index = 0;

	// update connectivity
	int nHalfEdges2 = (int)mesh.halfEdges.size();
	for (int j = nHalfEdges1; j < nHalfEdges2; j++) {
		int i = j == nHalfEdges1 ? nHalfEdges2 - 1 : j - 1;
		int k = j == nHalfEdges2 - 1 ? nHalfEdges1 : j + 1;

		HalfEdgeIter newPrev = mesh.halfEdges.begin() + i;
		HalfEdgeIter newHe = mesh.halfEdges.begin() + j;
		HalfEdgeIter newNext = mesh.halfEdges.begin() + k;
		HalfEdgeIter flip = newHe->flip;

		// update vertex connectivity
		VertexIter v = newHe->vertex;
		v->he = newHe;
		HalfEdgeIter h = flip->next;
		do {
			h->vertex = v;

			if (h->edge->onCut) break;
			h = h->flip->next;
		} while (true);

		// update edge connectivity
		EdgeIter e = newHe->edge;
		e->he = flip;
		e->onGenerator = true;

		// update face connectivity
		newHe->face = newF;

		// update halfedge connectivity
		flip->flip = newHe;
		newHe->prev = newPrev;
		newHe->next = newNext;
		newHe->onBoundary = true;
	}

	// assign halfedge edges and remove cut label from all edges
	for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		e->he->edge = e;
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
			HalfEdgeCIter he = e->he;

			// track vertices back to the root
			std::vector<EdgeIter> temp1;
			VertexCIter u = he->vertex;
			while (primalParent[u] != u) {
				VertexCIter v = primalParent[u];
				temp1.push_back(sharedEdge(u, v));
				u = v;
			}

			std::vector<EdgeIter> temp2;
			u = he->flip->vertex;
			while (primalParent[u] != u) {
				VertexCIter v = primalParent[u];
				temp2.push_back(sharedEdge(u, v));
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
