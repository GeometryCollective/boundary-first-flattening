#include "Face.h"
#include "Corner.h"

extern vector<HalfEdge> isolated;

inline Vertex::Vertex():
inNorthPoleVicinity(false),
index(-1)
{

}

inline bool Vertex::isIsolated() const
{
    return he == isolated.begin();
}

inline bool Vertex::onBoundary(bool checkIfOnCut) const
{
    if (inNorthPoleVicinity) return true;

    HalfEdgeCIter h = he;
    do {
        if (h->onBoundary) return true;
        if (checkIfOnCut && h->edge->onCut) return true;

        h = h->flip->next;
    } while (h != he);

    return false;
}

inline WedgeIter Vertex::wedge() const
{
    HalfEdgeCIter h = he;
    while (h->onBoundary || !h->next->wedge()->isReal()) h = h->flip->next;

    return h->next->wedge();
}

inline int Vertex::degree() const
{
    int k = 0;
    HalfEdgeCIter h = he;
    do {
        k++;

        h = h->flip->next;
    } while (h != he);

    return k;
}

inline Vector Vertex::normal() const
{
    Vector n;
    HalfEdgeCIter h = he;
    do {
        if (!h->onBoundary) n += h->face->normal(false)*h->next->corner->angle();

        h = h->flip->next;
    } while (h != he);

    n.normalize();

    return n;
}

inline double Vertex::angleDefect() const
{
    double sum = 0.0;
    if (onBoundary()) return sum;

    HalfEdgeCIter h = he;
    do {
        sum += h->next->corner->angle();

        h = h->flip->next;
    } while (h != he);

    return 2*M_PI - sum;
}

inline double Vertex::exteriorAngle() const
{
    double sum = 0.0;
    if (!onBoundary()) return sum;

    HalfEdgeCIter h = he;
    do {
        if (!h->onBoundary) sum += h->next->corner->angle();

        h = h->flip->next;
    } while (h != he);

    return M_PI - sum;
}
