#include "HoleFiller.h"

namespace bff {

bool HoleFiller::fill(Mesh& mesh, bool fillAll)
{
	// find longest boundary loop
	double loopLength;
	std::vector<std::vector<HalfEdgeIter>> boundaryHalfEdges;
	int longestLoop = longestBoundaryLoop(loopLength, boundaryHalfEdges, mesh.boundaries);
	if (loopLength < 0.5*mesh.diameter()) fillAll = true;

	// fill all holes (except possibly the longest)
	for (int i = 0; i < (int)mesh.boundaries.size(); i++) {
		if (fillAll || i != longestLoop) fill(boundaryHalfEdges[i], mesh);
	}

	// update mesh boundary array
	mesh.boundaries.clear();
	if (!fillAll) {
		BoundaryIter newB = mesh.boundaries.emplace(mesh.boundaries.end(), Face(&mesh));
		std::vector<HalfEdgeIter>& boundary = boundaryHalfEdges[longestLoop];
		newB->setHalfEdge(boundary[0]);
		newB->index = 0;

		for (int i = 0; i < (int)boundary.size(); i++) {
			boundary[i]->setFace(newB);
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

	for (BoundaryCIter b = boundaries.begin(); b != boundaries.end(); b++) {
		double length = 0;
		HalfEdgeIter he = b->halfEdge();
		do {
			boundaryHalfEdges[index].emplace_back(he);
			length += he->edge()->length();

			he = he->next();
		} while (he != b->halfEdge());

		if (length > loopLength) {
			loopLength = length;
			longestLoop = index;
		}

		index++;
	}

	return longestLoop;
}

void HoleFiller::fill(const std::vector<HalfEdgeIter>& boundaryHalfEdges, Mesh& mesh)
{
	int n = (int)boundaryHalfEdges.size();
	int nEdges = (int)mesh.edges.size();
	int nHalfEdges = (int)mesh.halfEdges.size();
	int nFaces = (int)mesh.faces.size();
	int nCorners = (int)mesh.corners.size();

	// inserting halfedges, edges, faces and corners *should* not invalidate iterators as
	// extra space was reserved in the preallocateElements function in MeshIO
	for (int i = 0; i < n - 3; i++) {
		EdgeIter e = mesh.edges.emplace(mesh.edges.end(), Edge(&mesh));
		HalfEdgeIter he = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
		HalfEdgeIter flip = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));

		e->index = nEdges + i;
		he->index = nHalfEdges + 2*i;
		flip->index = nHalfEdges + 2*i + 1;

		e->isCuttable = false;
		he->onBoundary = false;
		flip->onBoundary = false;

		e->setHalfEdge(he);
		he->setEdge(e);
		flip->setEdge(e);
		he->setFlip(flip);
		flip->setFlip(he);
	}

	for (int i = 0; i < n - 2; i++) {
		FaceIter f = mesh.faces.emplace(mesh.faces.end(), Face(&mesh));
		f->fillsHole = true;
		f->index = nFaces + i;

		for (int j = 0; j < 3; j++) {
			CornerIter c = mesh.corners.emplace(mesh.corners.end(), Corner(&mesh));
			c->index = nCorners + 3*i + j;
		}
	}

	// update connectivity
	VertexIter rootVertex = boundaryHalfEdges[0]->vertex();
	for (int i = 0; i < n; i++) {
		HalfEdgeIter he = boundaryHalfEdges[i];

		if (n == 3) {
			FaceIter newF = mesh.faces.begin() + nFaces;
			CornerIter newC = mesh.corners.begin() + nCorners + i;

			he->onBoundary = false;

			newF->setHalfEdge(he);
			he->setFace(newF);

			newC->setHalfEdge(he);
			he->setCorner(newC);

		} else {
			if (i == 0) {
				HalfEdgeIter next = he->next();
				HalfEdgeIter newHe = mesh.halfEdges.begin() + nHalfEdges;
				FaceIter newF = mesh.faces.begin() + nFaces;
				CornerIter newC1 = mesh.corners.begin() + nCorners;
				CornerIter newC2 = mesh.corners.begin() + nCorners + 1;
				CornerIter newC3 = mesh.corners.begin() + nCorners + 2;

				he->onBoundary = false;
				next->onBoundary = false;

				he->setPrev(newHe);
				next->setNext(newHe);
				newHe->setNext(he);
				newHe->setPrev(next);

				newF->setHalfEdge(he);
				he->setFace(newF);
				next->setFace(newF);
				newHe->setFace(newF);

				newC3->setHalfEdge(he);
				newC1->setHalfEdge(next);
				newC2->setHalfEdge(newHe);
				he->setCorner(newC3);
				next->setCorner(newC1);
				newHe->setCorner(newC2);

				newHe->setVertex(next->flip()->vertex());

			} else if (i > 1 && i < n - 2) {
				HalfEdgeIter newHe1 = mesh.halfEdges.begin() + (nHalfEdges + 2*(i - 2) + 1);
				HalfEdgeIter newHe2 = mesh.halfEdges.begin() + (nHalfEdges + 2*(i - 2) + 2);
				FaceIter newF = mesh.faces.begin() + (nFaces + i - 1);
				CornerIter newC1 = mesh.corners.begin() + (nCorners + 3*(i - 1));
				CornerIter newC2 = mesh.corners.begin() + (nCorners + 3*(i - 1) + 1);
				CornerIter newC3 = mesh.corners.begin() + (nCorners + 3*(i - 1) + 2);

				he->onBoundary = false;

				he->setNext(newHe2);
				newHe2->setNext(newHe1);
				newHe1->setNext(he);
				he->setPrev(newHe1);
				newHe1->setPrev(newHe2);
				newHe2->setPrev(he);

				newF->setHalfEdge(he);
				he->setFace(newF);
				newHe1->setFace(newF);
				newHe2->setFace(newF);

				newC3->setHalfEdge(he);
				newC1->setHalfEdge(newHe2);
				newC2->setHalfEdge(newHe1);
				he->setCorner(newC3);
				newHe2->setCorner(newC1);
				newHe1->setCorner(newC2);

				newHe1->setVertex(rootVertex);
				newHe2->setVertex(he->flip()->vertex());

			} else if (i == n - 1) {
				HalfEdgeIter prev = he->prev();
				HalfEdgeIter newHe = mesh.halfEdges.begin() + (nHalfEdges + 2*(i - 3) + 1);
				FaceIter newF = mesh.faces.begin() + (nFaces + i - 2);
				CornerIter newC1 = mesh.corners.begin() + (nCorners + 3*(i - 2));
				CornerIter newC2 = mesh.corners.begin() + (nCorners + 3*(i - 2) + 1);
				CornerIter newC3 = mesh.corners.begin() + (nCorners + 3*(i - 2) + 2);

				he->onBoundary = false;
				prev->onBoundary = false;

				he->setNext(newHe);
				prev->setPrev(newHe);
				newHe->setNext(prev);
				newHe->setPrev(he);

				newF->setHalfEdge(he);
				he->setFace(newF);
				prev->setFace(newF);
				newHe->setFace(newF);

				newC3->setHalfEdge(he);
				newC1->setHalfEdge(newHe);
				newC2->setHalfEdge(prev);
				he->setCorner(newC3);
				newHe->setCorner(newC1);
				prev->setCorner(newC2);

				newHe->setVertex(rootVertex);
			}
		}
	}
}

} // namespace bff
