#pragma once

#include "Types.h"

class HalfEdge {
public:
    // constructor
    HalfEdge();

    // next halfegde (in CCW order) associated with this halfedge's face
    HalfEdgeIter next;

    // prev halfegde associated with this halfedge's face
    HalfEdgeIter prev;

    // other halfedge associated with this halfedge's edge
    HalfEdgeIter flip;

    // vertex at the base of this halfedge
    VertexIter vertex;

    // edge associated with this halfedge
    EdgeIter edge;

    // face associated with this halfedge
    FaceIter face;

    // corner opposite to this halfedge. Undefined if this halfedge is on the boundary
    CornerIter corner;

    // id between 0 and |H|-1
    int index;

    // boolean flag to indicate if halfedge is on the boundary
    bool onBoundary;

    // returns wedge (a.k.a. corner) associated with this halfedge
    WedgeIter wedge() const;

    // returns cotan weight associated with this halfedge
    double cotan() const;
};

#include "HalfEdge.inl"
