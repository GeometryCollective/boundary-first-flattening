#include "bff/mesh/Corner.h"
#include "bff/mesh/Mesh.h"

namespace bff {

Corner::Corner(Mesh *mesh_):
inNorthPoleVicinity(false),
index(-1),
halfEdgeIndex(-1),
mesh(mesh_)
{

}

Corner::Corner(const Corner& c):
uv(c.uv),
inNorthPoleVicinity(c.inNorthPoleVicinity),
knot(c.knot),
index(c.index),
halfEdgeIndex(c.halfEdgeIndex),
mesh(c.mesh)
{

}

HalfEdgeIter Corner::halfEdge() const
{
	return mesh->halfEdges.begin() + halfEdgeIndex;
}

void Corner::setHalfEdge(HalfEdgeCIter he)
{
	halfEdgeIndex = he->index;
}

VertexIter Corner::vertex() const
{
	return halfEdge()->prev()->vertex();
}

FaceIter Corner::face() const
{
	return halfEdge()->face();
}

CornerIter Corner::next() const
{
	return halfEdge()->next()->corner();
}

CornerIter Corner::prev() const
{
	return halfEdge()->prev()->corner();
}

WedgeIter Wedge::nextWedge() const
{
	bool noCut = true;
	HalfEdgeCIter h = halfEdge()->prev();
	do {
		if (h->edge()->onCut) {
			noCut = false;
			break;
		}

		h = h->flip()->next();
	} while (!h->onBoundary);

	return noCut ? h->prev()->flip()->prev()->corner() :
				   h->prev()->corner();
}

void Corner::setMesh(Mesh *mesh_)
{
	mesh = mesh_;
}

bool Corner::isReal() const
{
	return !inNorthPoleVicinity;
}

} // namespace bff
