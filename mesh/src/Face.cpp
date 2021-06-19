#include "Face.h"
#include "Mesh.h"

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

Vector Face::normal(bool normalize) const
{
	if (!isReal()) return Vector();

	const Vector& a = halfEdge()->vertex()->position;
	const Vector& b = halfEdge()->next()->vertex()->position;
	const Vector& c = halfEdge()->prev()->vertex()->position;

	Vector n = cross(b - a, c - a);
	if (normalize) n.normalize();

	return n;
}

double Face::area() const
{
	return 0.5*normal(false).norm();
}

Vector Face::centroidUV() const
{
	if (!isReal()) return Vector();

	const Vector& a = halfEdge()->next()->wedge()->uv;
	const Vector& b = halfEdge()->prev()->wedge()->uv;
	const Vector& c = halfEdge()->wedge()->uv;

	return (a + b + c)/3.0;
}

double Face::areaUV() const
{
	if (!isReal()) return 0.0;

	const Vector& a = halfEdge()->next()->wedge()->uv;
	const Vector& b = halfEdge()->prev()->wedge()->uv;
	const Vector& c = halfEdge()->wedge()->uv;

	Vector n = cross(b - a, c - a);
	return 0.5*n.norm();
}

bool Face::isReal() const
{
	return !halfEdge()->onBoundary && !inNorthPoleVicinity;
}

} // namespace bff
