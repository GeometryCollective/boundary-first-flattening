#include <complex>

// given a time between 0 and 1, evaluates a cubic polynomial with
// the given endpoint and tangent values at the beginning (0) and
// end (1) of the interval
inline double cubicSplineUnitInterval(double p1, double p2,
									  double m1, double m2,
									  double t)
{
	double t2 = t*t;
	double t3 = t*t*t;

	return (2.0*t3 - 3.0*t2 + 1.0)*p1 + (t3 - 2.0*t2 + t)*m1 + (-2.0*t3 + 3.0*t2)*p2 + (t3 - t2)*m2;
}

inline double Spline::evaluate(double t) const
{
	if (knots.size() == 0) return 0.0;
	else if (knots.size() == 1) return knots.begin()->second;

	// determine knot positions
	KnotCIter k0, k1, k2, k3;
	k2 = knots.upper_bound(t);
	if (k2 == knots.end()) k2 = knots.begin();

	k3 = k2;
	k3++;
	if (k3 == knots.end()) k3 = knots.begin();

	if (k2 == knots.begin()) k1 = knots.end();
	else k1 = k2;
	k1--;

	if (k1 == knots.begin()) k0 = knots.end();
	else k0 = k1;
	k0--;

	double p0 = k0->second;
	double p1 = k1->second;
	double p2 = k2->second;
	double p3 = k3->second;

	// compute knot tangents with catmull rom scheme
	double m1 = 0.5*(p2 - p0);
	double m2 = 0.5*(p3 - p1);

	// normalize time
	std::complex<double> z(cos(t), sin(t));
	std::complex<double> z1(cos(k1->first), sin(k1->first));
	std::complex<double> z2(cos(k2->first), sin(k2->first));
	double normalizedTime = arg(z/z1)/arg(z2/z1);

	return cubicSplineUnitInterval(p1, p2, m1, m2, normalizedTime);
}

inline KnotIter Spline::addKnot(double t, double p)
{
	KnotIter k = knots.find(t);
	if (k != knots.end()) {
		k->second = p;
		return k;
	}

	return (knots.emplace(std::make_pair(t, p))).first;
}

inline bool Spline::removeKnot(double t, double tolerance)
{
	if (knots.size() == 0) return false;

	KnotCIter k2 = knots.lower_bound(t);

	KnotCIter k1 = k2;
	k1--;
	if (k2 == knots.end()) k2 = k1;

	double d1 = std::abs(k1->first - t);
	double d2 = std::abs(k2->first - t);

	if (d1 < tolerance && d1 < d2) {
		knots.erase(k1);
		return true;
	}

	if (d2 < tolerance && d2 < d1) {
		knots.erase(k2);
		return true;
	}

	return false;
}

inline void Spline::reset()
{
	for (KnotIter knot = knots.begin(); knot != knots.end(); knot++) {
		knot->second = 0.0;
	}
}
