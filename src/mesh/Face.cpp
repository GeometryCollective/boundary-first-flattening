#include "bff/mesh/Face.h"
#include "bff/mesh/Mesh.h"

namespace bff {

Face::Face(Mesh *mesh_):
fillsHole(false),
inNorthPoleVicinity(false),
index(-1),
halfEdgeIndex(-1),
mesh(mesh_)
{

}

Face::Face(const Face& f):
fillsHole(f.fillsHole),
inNorthPoleVicinity(f.inNorthPoleVicinity),
index(f.index),
halfEdgeIndex(f.halfEdgeIndex),
mesh(f.mesh)
{

}

HalfEdgeIter Face::halfEdge() const
{
	return mesh->halfEdges.begin() + halfEdgeIndex;
}

void Face::setHalfEdge(HalfEdgeCIter he)
{
	halfEdgeIndex = he->index;
}

void Face::setMesh(Mesh *mesh_)
{
	mesh = mesh_;
}

bool Face::isReal() const
{
	return !halfEdge()->onBoundary && !inNorthPoleVicinity;
}

} // namespace bff
