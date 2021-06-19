#include "Vertex.h"
#include "Mesh.h"

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

Vector Vertex::normal() const
{
	Vector n;
	HalfEdgeCIter h = halfEdge();
	do {
		if (!h->onBoundary) n += h->face()->normal(false)*h->next()->corner()->angle();

		h = h->flip()->next();
	} while (h != halfEdge());

	n.normalize();

	return n;
}

double Vertex::angleDefect() const
{
	double sum = 0.0;
	if (onBoundary()) return sum;

	HalfEdgeCIter h = halfEdge();
	do {
		sum += h->next()->corner()->angle();

		h = h->flip()->next();
	} while (h != halfEdge());

	return 2*M_PI - sum;
}

double Vertex::exteriorAngle() const
{
	double sum = 0.0;
	if (!onBoundary()) return sum;

	HalfEdgeCIter h = halfEdge();
	do {
		if (!h->onBoundary) sum += h->next()->corner()->angle();

		h = h->flip()->next();
	} while (h != halfEdge());

	return M_PI - sum;
}

} // namespace bff
