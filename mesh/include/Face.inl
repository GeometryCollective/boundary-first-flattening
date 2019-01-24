namespace bff {

inline Face::Face():
fillsHole(false),
inNorthPoleVicinity(false),
index(-1)
{

}

inline Vector Face::normal(bool normalize) const
{
	if (!isReal()) return Vector();

	const Vector& a = he->vertex->position;
	const Vector& b = he->next->vertex->position;
	const Vector& c = he->prev->vertex->position;

	Vector n = cross(b - a, c - a);
	if (normalize) n.normalize();

	return n;
}

inline double Face::area() const
{
	return 0.5*normal(false).norm();
}

inline bool Face::isReal() const
{
	return !he->onBoundary && !inNorthPoleVicinity;
}

} // namespace bff
