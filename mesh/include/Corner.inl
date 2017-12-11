#include "Edge.h"

inline Corner::Corner():
inNorthPoleVicinity(false),
index(-1)
{

}

inline VertexIter Corner::vertex() const
{
    return he->prev->vertex;
}

inline FaceIter Corner::face() const
{
    return he->face;
}

inline CornerIter Corner::next() const
{
    return he->next->corner;
}

inline CornerIter Corner::prev() const
{
    return he->prev->corner;
}

inline double Corner::angle() const
{
    const Vector& a = he->prev->vertex->position;
    const Vector& b = he->vertex->position;
    const Vector& c = he->next->vertex->position;

    Vector u = b - a;
    u.normalize();

    Vector v = c - a;
    v.normalize();

    return acos(max(-1.0, min(1.0, dot(u, v))));
}

inline bool Corner::isReal() const
{
    return !inNorthPoleVicinity;
}

// Note: wedge must lie on the (possibly cut) boundary
inline double exteriorAngle(WedgeCIter w)
{
    HalfEdgeCIter he = w->he->prev;

    double sum = 0.0;
    if (!he->vertex->onBoundary()) return sum;

    do {
        sum += he->next->corner->angle();
        if (he->edge->onCut) break;

        he = he->flip->next;
    } while (!he->onBoundary);

    return M_PI - sum;
}

// returns next wedge (in CCW order) along the (possibly cut) boundary
inline WedgeIter nextWedge(WedgeCIter w)
{
    bool noCut = true;
    HalfEdgeCIter he = w->he->prev;
    do {
        if (he->edge->onCut) {
            noCut = false;
            break;
        }

        he = he->flip->next;
    } while (!he->onBoundary);

    return noCut ? he->prev->flip->prev->corner : he->prev->corner;
}

inline double scaling(WedgeCIter w)
{
    WedgeCIter next = nextWedge(w);

    Vector a = w->prev()->uv;
    Vector b = w->uv;
    Vector c = next->uv;

    double lij = (b - a).norm();
    double ljk = (c - b).norm();

    double uij = log(lij/w->he->next->edge->length());
    double ujk = log(ljk/next->he->next->edge->length());

    return (lij*uij + ljk*ujk)/(lij + ljk);
}

inline Vector tangent(WedgeCIter w)
{
    Vector a = w->prev()->uv;
    Vector b = w->uv;
    Vector c = nextWedge(w)->uv;

    Vector Tij = b - a;
    Tij.normalize();

    Vector Tjk = c - b;
    Tjk.normalize();

    Vector T = Tij + Tjk;
    T.normalize();

    return T;
}

struct WedgeIterHash {
    size_t operator()(WedgeCIter w) const {
        return (size_t)w->index;
    }
};
