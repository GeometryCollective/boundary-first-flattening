#pragma once

#include "Types.h"

class Edge {
public:
    // constructor
    Edge();

    // one of the halfedges associated with this edge
    HalfEdgeIter he;

    // boolean flag to indicate if edge is on a cut
    bool onCut;

    // id between 0 and |E|-1
    int index;

    // returns edge length
    double length() const;

    // returns cotan weight associated with this edge
    double cotan() const;

    // checks if this edge is on the boundary
    bool onBoundary() const;
};

#include "Edge.inl"
