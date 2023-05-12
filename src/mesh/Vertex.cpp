#include "bff/mesh/Vertex.h"
#include "bff/mesh/Mesh.h"

namespace bff {

Vertex::Vertex(Mesh *mesh_):
inNorthPoleVicinity(false),
index(-1),
referenceIndex(-1),
halfEdgeIndex(-1),
mesh(mesh_)
{

}

Vertex::Vertex(const Vertex& v):
position(v.position),
inNorthPoleVicinity(v.inNorthPoleVicinity),
index(v.index),
referenceIndex(v.referenceIndex),
halfEdgeIndex(v.halfEdgeIndex),
mesh(v.mesh)
{

}

HalfEdgeIter Vertex::halfEdge() const
{
	return mesh->halfEdges.begin() + halfEdgeIndex;
}

void Vertex::setHalfEdge(HalfEdgeCIter he)
{
	halfEdgeIndex = he->index;
}

WedgeIter Vertex::wedge() const
{
	HalfEdgeCIter h = halfEdge();
	while (h->onBoundary || !h->next()->wedge()->isReal()) {
		h = h->flip()->next();
	}

	return h->next()->wedge();
}

void Vertex::setMesh(Mesh *mesh_)
{
	mesh = mesh_;
}

bool Vertex::onBoundary(bool checkIfOnCut) const
{
	if (inNorthPoleVicinity) return true;

	HalfEdgeCIter h = halfEdge();
	do {
		if (h->onBoundary) return true;
		if (checkIfOnCut && h->edge()->onCut) return true;

		h = h->flip()->next();
	} while (h != halfEdge());

	return false;
}

bool Vertex::isIsolated() const
{
	return halfEdgeIndex == -1;
}

bool Vertex::insideHole() const
{
	HalfEdgeCIter h = halfEdge();
	do {
		if (!h->face()->fillsHole) {
			return false;
		}

		h = h->flip()->next();
	} while (h != halfEdge());

	return true;
}

int Vertex::degree() const
{
	int k = 0;
	HalfEdgeCIter h = halfEdge();
	do {
		k++;

		h = h->flip()->next();
	} while (h != halfEdge());

	return k;
}

} // namespace bff
