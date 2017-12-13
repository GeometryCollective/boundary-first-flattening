#include "Cutter.h"
#include "DisjointSets.h"
#include <queue>
#include <tuple>
#include <functional>

typedef tuple<double, VertexIter, HalfEdgeIter> VertexEntry;

void Cutter::cut(const vector<VertexIter>& cones, Mesh& mesh)
{
    // initialize data structures
    int nV = (int)mesh.vertices.size();
    DisjointSets ds(nV);
    vector<HalfEdgeIter> parent(nV, mesh.halfEdges.end());
    priority_queue<VertexEntry, vector<VertexEntry>, greater<VertexEntry>> pq;

    // merge each boundary loop
    for (BoundaryCIter b = mesh.boundaries.begin(); b != mesh.boundaries.end(); b++) {
        bool first = true;
        HalfEdgeIter he = b->he;
        do {
            if (first) first = false;
            else ds.merge(he->vertex->index, he->next->vertex->index);

            // add initial entries to pq
            HalfEdgeIter vHe = he;
            do {
                EdgeCIter e = vHe->edge;
                if (!e->onBoundary()) pq.push(VertexEntry(e->length(), vHe->flip->vertex, vHe));

                vHe = vHe->flip->next;
            } while (vHe != he);

            he = he->next;
        } while (he != b->he);

        // mark set
        ds.mark(he->vertex->index);
    }

    // add cone neighbors to pq
    for (int i = 0; i < (int)cones.size(); i++) {
        HalfEdgeIter he = cones[i]->he;
        do {
            pq.push(VertexEntry(he->edge->length(), he->flip->vertex, he));

            he = he->flip->next;
        } while (he != cones[i]->he);

        // mark set
        ds.mark(cones[i]->index);
    }

    // construct approximate steiner tree
    while (!pq.empty()) {
        VertexEntry entry = pq.top();
        pq.pop();

        double weight = get<0>(entry);
        HalfEdgeIter he = get<2>(entry);
        VertexIter v1 = get<1>(entry);
        VertexIter v2 = he->vertex;

        if (ds.find(v1->index) != ds.find(v2->index)) {
            // if merging two marked sets, then mark edges that connect these two regions as cut edges
            if (ds.isMarked(v1->index) && ds.isMarked(v2->index)) {
                // one side
                HalfEdgeIter currHe = parent[v1->index];
                while (currHe != mesh.halfEdges.end()) {
                    currHe->edge->onCut = true;
                    currHe = parent[currHe->vertex->index];
                }

                // bridge
                he->edge->onCut = true;

                // other side
                currHe = parent[v2->index];
                while (currHe != mesh.halfEdges.end()) {
                    currHe->edge->onCut = true;
                    currHe = parent[currHe->vertex->index];
                }
            }

            // record potential parent and merge sets
            parent[v1->index] = he;
            ds.merge(v1->index, v2->index);

            // add neighbors
            HalfEdgeIter vHe = v1->he;
            do {
                pq.push(VertexEntry(vHe->edge->length() + weight, vHe->flip->vertex, vHe));

                vHe = vHe->flip->next;
            } while (vHe != v1->he);
        }
    }
}

void Cutter::glue(Mesh& mesh)
{
    for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        e->onCut = false;
    }
}
