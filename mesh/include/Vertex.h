#pragma once

#include "Types.h"
#include "Spline.h"

class Vertex {
public:
    // constructor
    Vertex();

    // outgoing halfedge
    HalfEdgeIter he;

    // position
    Vector position;

    // flag to indicate whether this vertex is a neighbor of or is
    // the north pole of a stereographic projection from the disk to a sphere
    bool inNorthPoleVicinity;

    // id between 0 and |V|-1
    int index;

    // checks if this vertex is isolated
    bool isIsolated() const;

    // checks if this vertex is on the boundary
    bool onBoundary(bool checkIfOnCut = true) const;

    // returns one of the wedges (a.k.a. corner) associated with this vertex
    WedgeIter wedge() const;

    // returns degree
    int degree() const;

    // returns angle weighted average of adjacent face normals
    Vector normal() const;

    // returns 2π minus sum of incident angles. Note: only valid for interior vertices
    double angleDefect() const;

    // returns exterior angle. Note: only valid for boundary vertices
    double exteriorAngle() const;
};

#include "Vertex.inl"
