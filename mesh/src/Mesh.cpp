#include "Mesh.h"
#include <limits>

namespace bff {

Mesh::Mesh():
radius(0.0),
cm(0.0, 0.0, 0.0),
status(ErrorCode::ok)
{

}

Mesh::Mesh(const Mesh& mesh):
radius(mesh.radius),
cm(mesh.cm),
status(mesh.status)
{
	// initialize halfEdges
	halfEdges.reserve(mesh.halfEdges.size());
	for (HalfEdgeCIter h = mesh.halfEdges.begin(); h != mesh.halfEdges.end(); h++) {
		HalfEdgeIter hNew = halfEdges.emplace(halfEdges.end(), HalfEdge(*h));
		hNew->setMesh(this);
	}

	// initialize vertices
	vertices.reserve(mesh.vertices.size());
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		VertexIter vNew = vertices.emplace(vertices.end(), Vertex(*v));
		vNew->setMesh(this);
	}

	// initialize edges
	edges.reserve(mesh.edges.size());
	for (EdgeCIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		EdgeIter eNew = edges.emplace(edges.end(), Edge(*e));
		eNew->setMesh(this);
	}

	// initialize faces
	faces.reserve(mesh.faces.size());
	for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
		FaceIter fNew = faces.emplace(faces.end(), Face(*f));
		fNew->setMesh(this);
	}

	// initialize corners
	corners.reserve(mesh.corners.size());
	for (CornerCIter c = mesh.corners.begin(); c != mesh.corners.end(); c++) {
		CornerIter cNew = corners.emplace(corners.end(), Corner(*c));
		cNew->setMesh(this);
	}

	// initialize boundaries
	boundaries.reserve(mesh.boundaries.size());
	for (BoundaryCIter b = mesh.boundaries.begin(); b != mesh.boundaries.end(); b++) {
		BoundaryIter bNew = boundaries.emplace(boundaries.end(), Face(*b));
		bNew->setMesh(this);
	}
}

int Mesh::eulerCharacteristic() const
{
	return (int)(vertices.size() - edges.size() + faces.size());
}

double Mesh::diameter() const
 {
	double maxLimit = std::numeric_limits<double>::max();
	double minLimit = std::numeric_limits<double>::min();
	Vector minBounds(maxLimit, maxLimit, maxLimit);
	Vector maxBounds(minLimit, minLimit, minLimit);

	for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
		const Vector& p = v->position;

		minBounds.x = std::min(p.x, minBounds.x);
		minBounds.y = std::min(p.y, minBounds.y);
		minBounds.z = std::min(p.z, minBounds.z);
		maxBounds.x = std::max(p.x, maxBounds.x);
		maxBounds.y = std::max(p.y, maxBounds.y);
		maxBounds.z = std::max(p.z, maxBounds.z);
	}

	return (maxBounds - minBounds).norm();
 }

CutPtrSet Mesh::cutBoundary()
{
	if (boundaries.size() == 0) {
		// if there is no boundary, initialize the iterator with the first edge
		// on the cut
		for (EdgeCIter e = edges.begin(); e != edges.end(); e++) {
			if (e->onCut) return CutPtrSet(e->halfEdge());
		}

		return CutPtrSet();
	}

	return CutPtrSet(boundaries[0].halfEdge());
}

std::vector<Wedge>& Mesh::wedges()
{
	return corners;
}

const std::vector<Wedge>& Mesh::wedges() const
{
	return corners;
}

} // namespace bff
