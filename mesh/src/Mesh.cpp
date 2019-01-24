#include "Mesh.h"
#include <limits>

namespace bff {

Mesh::Mesh():
radius(0.0)
{

}

Mesh::Mesh(const Mesh& mesh):
radius(mesh.radius)
{
	// allocate halfEdges
	halfEdges.reserve(mesh.halfEdges.size());
	for (HalfEdgeCIter h = mesh.halfEdges.begin(); h != mesh.halfEdges.end(); h++) {
		halfEdges.insert(halfEdges.end(), HalfEdge());
	}

	// initialize vertices
	vertices.reserve(mesh.vertices.size());
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		VertexIter vNew = vertices.insert(vertices.end(), Vertex());
		vNew->he = halfEdges.begin() + v->he->index;
		vNew->position = v->position;
		vNew->inNorthPoleVicinity = v->inNorthPoleVicinity;
		vNew->index = v->index;
		vNew->referenceIndex = v->referenceIndex;
	}

	// initialize edges
	edges.reserve(mesh.edges.size());
	for (EdgeCIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		EdgeIter eNew = edges.insert(edges.end(), Edge());
		eNew->he = halfEdges.begin() + e->he->index;
		eNew->onGenerator = e->onGenerator;
		eNew->onCut = e->onCut;
		eNew->isCuttable = e->isCuttable;
		eNew->index = e->index;
	}

	// initialize faces
	faces.reserve(mesh.faces.size());
	for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
		FaceIter fNew = faces.insert(faces.end(), Face());
		fNew->he = halfEdges.begin() + f->he->index;
		fNew->fillsHole = f->fillsHole;
		fNew->inNorthPoleVicinity = f->inNorthPoleVicinity;
		fNew->index = f->index;
	}

	// initialize corners
	corners.reserve(mesh.corners.size());
	for (CornerCIter c = mesh.corners.begin(); c != mesh.corners.end(); c++) {
		CornerIter cNew = corners.insert(corners.end(), Corner());
		cNew->he = halfEdges.begin() + c->he->index;
		cNew->uv = c->uv;
		cNew->inNorthPoleVicinity = c->inNorthPoleVicinity;
		cNew->index = c->index;
	}

	// initialize boundaries
	boundaries.reserve(mesh.boundaries.size());
	for (BoundaryCIter b = mesh.boundaries.begin(); b != mesh.boundaries.end(); b++) {
		BoundaryIter bNew = boundaries.insert(boundaries.end(), Face());
		bNew->he = halfEdges.begin() + b->he->index;
		bNew->fillsHole = b->fillsHole;
		bNew->inNorthPoleVicinity = b->inNorthPoleVicinity;
		bNew->index = b->index;
	}

	// initialize halfEdges
	for (HalfEdgeCIter h = mesh.halfEdges.begin(); h != mesh.halfEdges.end(); h++) {
		HalfEdgeIter hNew = halfEdges.begin() + h->index;
		hNew->next = halfEdges.begin() + h->next->index;
		hNew->prev = halfEdges.begin() + h->prev->index;
		hNew->flip = halfEdges.begin() + h->flip->index;
		hNew->vertex = vertices.begin() + h->vertex->index;
		hNew->edge = edges.begin() + h->edge->index;
		hNew->face = faces.begin() + h->face->index;
		if (!h->onBoundary) hNew->corner = corners.begin() + h->corner->index;
		hNew->index = h->index;
		hNew->onBoundary = h->onBoundary;
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
			if (e->onCut) return CutPtrSet(e->he);
		}

		return CutPtrSet();
	}

	return CutPtrSet(boundaries[0].he);
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
