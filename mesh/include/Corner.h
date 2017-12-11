#pragma once

#include "Types.h"

class Corner {
public:
    // constructor
    Corner();

    // halfedge opposite to this corner
    HalfEdgeIter he;

    // uv coordinates
    Vector uv;

    // flag to indicate whether this corner is contained in a face that is
    // incident to the north pole of a stereographic projection from the disk to a sphere
    bool inNorthPoleVicinity;

    // spline handle for direct editing
    KnotIter knot;

    // id between 0 and |C|-1
    int index;

    // returns vertex associated with this corner
    VertexIter vertex() const;

    // returns face associated with this corner
    FaceIter face() const;

    // returns next corner (in CCW order) associated with this corner's face
    CornerIter next() const;

    // returns previous corner (in CCW order) associated with this corner's face
    CornerIter prev() const;

    // returns angle (in radians) at this corner
    double angle() const;

    // checks if this corner is real
    bool isReal() const;
};

#include "Corner.inl"
