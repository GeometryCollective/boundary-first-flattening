#include "HoleFiller.h"

namespace bff {

bool HoleFiller::fill(Mesh& mesh, bool fillAll)
{
	// find longest boundary loop
	double loopLength;
	std::vector<std::vector<HalfEdgeIter>> boundaryHalfEdges;
	int longestLoop = longestBoundaryLoop(loopLength, boundaryHalfEdges, mesh.boundaries);
	if (loopLength < 0.5*mesh.diameter()) fillAll = true;

	// fill all holes
	int index = 0;
	for (FaceIter f = mesh.boundaries.begin(); f != mesh.boundaries.end(); f++) {
		if (longestLoop != index || fillAll) fill(f, boundaryHalfEdges[index], mesh);
		index++;
	}

	// update mesh boundary array
	mesh.boundaries.clear();
	if (!fillAll) {
		FaceIter newF = mesh.boundaries.insert(mesh.boundaries.end(), Face());
		std::vector<HalfEdgeIter>& boundary = boundaryHalfEdges[longestLoop];
		newF->he = boundary[0];
		newF->index = 0;

		for (int i = 0; i < (int)boundary.size(); i++) {
			boundary[i]->face = newF;
		}
	}

	return fillAll;
}

int HoleFiller::longestBoundaryLoop(double& loopLength,
									std::vector<std::vector<HalfEdgeIter>>& boundaryHalfEdges,
									const std::vector<Face>& boundaries)
{
	int index = 0;
	int longestLoop = -1;
	loopLength = 0.0;
	boundaryHalfEdges.resize(boundaries.size());

	for (FaceCIter f = boundaries.begin(); f != boundaries.end(); f++) {
		double length = 0;
		HalfEdgeIter he = f->he;
		do {
			boundaryHalfEdges[index].push_back(he);
			length += he->edge->length();

			he = he->next;
		} while (he != f->he);

		if (length > loopLength) {
			loopLength = length;
			longestLoop = index;
		}

		index++;
	}

	return longestLoop;
}

void HoleFiller::fill(FaceIter b, const std::vector<HalfEdgeIter>& boundaryHalfEdges, Mesh& mesh)
{
	int n = (int)boundaryHalfEdges.size();
	int nEdges = (int)mesh.edges.size();
	int nHalfEdges = (int)mesh.halfEdges.size();
	int nFaces = (int)mesh.faces.size();
	int nCorners = (int)mesh.corners.size();

	// inserting halfedges, edges, faces and corners *should* not invalidate iterators as
	// extra space was reserved in the preallocateElements function in MeshIO
	for (int i = 0; i < n - 3; i++) {
		EdgeIter e = mesh.edges.insert(mesh.edges.end(), Edge());
		HalfEdgeIter he = mesh.halfEdges.insert(mesh.halfEdges.end(), HalfEdge());
		HalfEdgeIter flip = mesh.halfEdges.insert(mesh.halfEdges.end(), HalfEdge());
		he->onBoundary = false;
		flip->onBoundary = false;

		e->isCuttable = false;
		e->he = he;
		he->edge = e;
		flip->edge = e;
		he->flip = flip;
		flip->flip = he;

		e->index = nEdges + i;
		he->index = nHalfEdges + 2*i;
		flip->index = nHalfEdges + 2*i + 1;
	}

	for (int i = 0; i < n - 2; i++) {
		FaceIter f = mesh.faces.insert(mesh.faces.end(), Face());
		f->fillsHole = true;
		f->index = nFaces + i;

		for (int j = 0; j < 3; j++) {
			CornerIter c = mesh.corners.insert(mesh.corners.end(), Corner());
			c->index = nCorners + 3*i + j;
		}
	}

	// update connectivity
	VertexIter rootVertex = boundaryHalfEdges[0]->vertex;
	for (int i = 0; i < (int)boundaryHalfEdges.size(); i++) {
		HalfEdgeIter he = boundaryHalfEdges[i];

		if (i == 0) {
			HalfEdgeIter next = he->next;
			HalfEdgeIter newHe = mesh.halfEdges.begin() + nHalfEdges;
			FaceIter newF = mesh.faces.begin() + nFaces;
			CornerIter newC1 = mesh.corners.begin() + nCorners;
			CornerIter newC2 = mesh.corners.begin() + nCorners + 1;
			CornerIter newC3 = mesh.corners.begin() + nCorners + 2;

			he->onBoundary = false;
			next->onBoundary = false;

			he->prev = newHe;
			next->next = newHe;
			newHe->next = he;
			newHe->prev = next;

			newF->he = he;
			he->face = newF;
			next->face = newF;
			newHe->face = newF;

			newC3->he = he;
			newC1->he = next;
			newC2->he = newHe;
			he->corner = newC3;
			next->corner = newC1;
			newHe->corner = newC2;

			newHe->vertex = next->flip->vertex;

		} else if (i > 1 && i < n - 2) {
			HalfEdgeIter newHe1 = mesh.halfEdges.begin() + (nHalfEdges + 2*(i - 2) + 1);
			HalfEdgeIter newHe2 = mesh.halfEdges.begin() + (nHalfEdges + 2*(i - 2) + 2);
			FaceIter newF = mesh.faces.begin() + (nFaces + i - 1);
			CornerIter newC1 = mesh.corners.begin() + (nCorners + 3*(i - 1));
			CornerIter newC2 = mesh.corners.begin() + (nCorners + 3*(i - 1) + 1);
			CornerIter newC3 = mesh.corners.begin() + (nCorners + 3*(i - 1) + 2);

			he->onBoundary = false;

			he->next = newHe2;
			newHe2->next = newHe1;
			newHe1->next = he;
			he->prev = newHe1;
			newHe1->prev = newHe2;
			newHe2->prev = he;

			newF->he = he;
			he->face = newF;
			newHe1->face = newF;
			newHe2->face = newF;

			newC3->he = he;
			newC1->he = newHe2;
			newC2->he = newHe1;
			he->corner = newC3;
			newHe2->corner = newC1;
			newHe1->corner = newC2;

			newHe1->vertex = rootVertex;
			newHe2->vertex = he->flip->vertex;

		} else if (i == n - 1) {
			HalfEdgeIter prev = he->prev;
			HalfEdgeIter newHe = mesh.halfEdges.begin() + (nHalfEdges + 2*(i - 3) + 1);
			FaceIter newF = mesh.faces.begin() + (nFaces + i - 2);
			CornerIter newC1 = mesh.corners.begin() + (nCorners + 3*(i - 2));
			CornerIter newC2 = mesh.corners.begin() + (nCorners + 3*(i - 2) + 1);
			CornerIter newC3 = mesh.corners.begin() + (nCorners + 3*(i - 2) + 2);

			he->onBoundary = false;
			prev->onBoundary = false;

			he->next = newHe;
			prev->prev = newHe;
			newHe->next = prev;
			newHe->prev = he;

			newF->he = he;
			he->face = newF;
			prev->face = newF;
			newHe->face = newF;

			newC3->he = he;
			newC1->he = newHe;
			newC2->he = prev;
			he->corner = newC3;
			newHe->corner = newC1;
			prev->corner = newC2;

			newHe->vertex = rootVertex;
		}
	}
}

} // namespace bff
