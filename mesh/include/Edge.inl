inline Edge::Edge():
onCut(false),
index(-1)
{

}

inline double Edge::length() const
{
    const Vector& a = he->vertex->position;
    const Vector& b = he->next->vertex->position;

    return (b - a).norm();
}

inline double Edge::cotan() const
{
    return 0.5*(he->cotan() + he->flip->cotan());
}

inline bool Edge::onBoundary() const
{
    return he->onBoundary || he->flip->onBoundary;
}
