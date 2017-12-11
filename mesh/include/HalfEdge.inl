#include "Vertex.h"

inline HalfEdge::HalfEdge():
index(-1)
{

}

inline WedgeIter HalfEdge::wedge() const
{
    return corner;
}

inline double HalfEdge::cotan() const
{
    if (onBoundary) return 0.0;

    const Vector& a = vertex->position;
    const Vector& b = next->vertex->position;
    const Vector& c = prev->vertex->position;

    Vector u = a - c;
    Vector v = b - c;

    return dot(u, v)/cross(u, v).norm();
}
