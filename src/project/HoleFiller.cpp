#include "bff/project/HoleFiller.h"

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
		double l = 0;
		HalfEdgeIter he = b->halfEdge();
		do {
			boundaryHalfEdges[index].emplace_back(he);
			l += length(he->edge());

			he = he->next();
		} while (he != b->halfEdge());

		if (l > loopLength) {
			loopLength = l;
			longestLoop = index;
		}

		index++;
	}

	return longestLoop;
}

void HoleFiller::fill(const std::vector<HalfEdgeIter>& boundaryHalfEdges, Mesh& mesh)
{
	int n = (int)boundaryHalfEdges.size();
    int nVertices = (int)mesh.vertices.size();
    int nEdges = (int)mesh.edges.size();
    int nHalfEdges = (int)mesh.halfEdges.size();
    int nFaces = (int)mesh.faces.size();
    int nCorners = (int)mesh.corners.size();

    // insert new vertex at the center of the hole
    VertexIter v = mesh.vertices.emplace(mesh.vertices.end(), Vertex(&mesh));
    v->position = Vector();
    v->index = nVertices;

    // insert new halfedges, edges and faces
    for (int i = 0; i < n; i++) {
        HalfEdgeIter bHe = boundaryHalfEdges[i];
        HalfEdgeIter bHePrev = bHe->prev();

        // insert new halfedges and update connectivity
        HalfEdgeIter he = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
        he->onBoundary = false;
        he->index = nHalfEdges + 2*i;

        HalfEdgeIter flip = mesh.halfEdges.emplace(mesh.halfEdges.end(), HalfEdge(&mesh));
        flip->onBoundary = false;
        flip->index = nHalfEdges + 2*i + 1;

        bHe->onBoundary = false;
        bHe->setPrev(he);
        he->setNext(bHe);
        he->setFlip(flip);
        bHePrev->setNext(flip);
        flip->setPrev(bHePrev);
        flip->setFlip(he);

        // insert new edge and update connectivity
        EdgeIter e = mesh.edges.emplace(mesh.edges.end(), Edge(&mesh));
        e->isCuttable = false;
        e->index = nEdges + i;

        e->setHalfEdge(he);
        he->setEdge(e);
        flip->setEdge(e);

        // insert new face and update connectivity
        FaceIter f = mesh.faces.emplace(mesh.faces.end(), Face(&mesh));
        f->fillsHole = true;
        f->index = nFaces + i;

        f->setHalfEdge(bHe);
        bHe->setFace(f);
        he->setFace(f);

        // update vertex connectivity
        v->position += bHe->vertex()->position;
        v->setHalfEdge(he);
        he->setVertex(v);
        flip->setVertex(bHe->vertex());
    }

    v->position /= n;

    // insert corners and update remaining connectivity
    for (int i = 0; i < n; i++) {
        HalfEdgeIter bHe = boundaryHalfEdges[i];
        HalfEdgeIter bHePrev = bHe->prev();
        HalfEdgeIter bHeNext = bHe->next();

        // insert new corners and update connectivity
        CornerIter c1 = mesh.corners.emplace(mesh.corners.end(), Corner(&mesh));
        CornerIter c2 = mesh.corners.emplace(mesh.corners.end(), Corner(&mesh));
        CornerIter c3 = mesh.corners.emplace(mesh.corners.end(), Corner(&mesh));
        c1->index = nCorners + 3*i;
        c2->index = nCorners + 3*i + 1;
        c3->index = nCorners + 3*i + 2;

        c1->setHalfEdge(bHe);
        c2->setHalfEdge(bHeNext);
        c3->setHalfEdge(bHePrev);
        bHe->setCorner(c1);
        bHeNext->setCorner(c2);
        bHePrev->setCorner(c3);

        // update remaining connectivity
        bHeNext->setFace(bHe->face());
        bHeNext->setNext(bHePrev);
        bHePrev->setPrev(bHeNext);
    }
}

} // namespace bff
