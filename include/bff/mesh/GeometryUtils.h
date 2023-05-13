#pragma once

#include "bff/mesh/HalfEdge.h"
#include "bff/mesh/Vertex.h"
#include "bff/mesh/Edge.h"
#include "bff/mesh/Face.h"
#include "bff/mesh/Corner.h"

namespace bff {

// returns the cotan weight associated with a halfedge
inline double cotan(HalfEdgeCIter h) {
	if (h->onBoundary) return 0.0;

	const Vector& a = h->vertex()->position;
	const Vector& b = h->next()->vertex()->position;
	const Vector& c = h->prev()->vertex()->position;

	Vector u = a - c;
	Vector v = b - c;

	double w = dot(u, v)/cross(u, v).norm();
	if (std::isinf(w) || std::isnan(w)) w = 0.0;
	return w;
}

// returns edge length
inline double length(EdgeCIter e) {
	HalfEdgeCIter h = e->halfEdge();
	const Vector& a = h->vertex()->position;
	const Vector& b = h->next()->vertex()->position;

	return (b - a).norm();
}

// returns face normal
inline Vector normal(FaceCIter f, bool normalize = true) {
	if (!f->isReal()) return Vector();

	HalfEdgeCIter h = f->halfEdge();
	const Vector& a = h->vertex()->position;
	const Vector& b = h->next()->vertex()->position;
	const Vector& c = h->prev()->vertex()->position;

	Vector n = cross(b - a, c - a);
	if (normalize) n.normalize();

	return n;
}

// returns face area
inline double area(FaceCIter f) {
	return 0.5*normal(f, false).norm();
}

// returns face centroid in uv plane
inline Vector centroidUV(FaceCIter f) {
	if (!f->isReal()) return Vector();

	HalfEdgeCIter h = f->halfEdge();
	const Vector& a = h->next()->wedge()->uv;
	const Vector& b = h->prev()->wedge()->uv;
	const Vector& c = h->wedge()->uv;

	return (a + b + c)/3.0;
}

// returns face area in uv plane
inline double areaUV(FaceCIter f) {
	if (!f->isReal()) return 0.0;

	HalfEdgeCIter h = f->halfEdge();
	const Vector& a = h->next()->wedge()->uv;
	const Vector& b = h->prev()->wedge()->uv;
	const Vector& c = h->wedge()->uv;

	Vector n = cross(b - a, c - a);
	return 0.5*n.norm();
}

// returns the angle (in radians) at a corner
inline double angle(CornerCIter c) {
	HalfEdgeCIter h = c->halfEdge();
	const Vector& p1 = h->prev()->vertex()->position;
	const Vector& p2 = h->vertex()->position;
	const Vector& p3 = h->next()->vertex()->position;

	Vector u = p2 - p1;
	u.normalize();

	Vector v = p3 - p1;
	v.normalize();

	return acos(std::max(-1.0, std::min(1.0, dot(u, v))));
}

// computes the exterior angle at a wedge; Note: wedge must lie on the (possibly cut) boundary
inline double exteriorAngle(WedgeCIter w) {
	HalfEdgeCIter h = w->halfEdge()->prev();

	double sum = 0.0;
	if (!h->vertex()->onBoundary()) return sum;

	do {
		sum += angle(h->next()->corner());
		if (h->edge()->onCut) break;

		h = h->flip()->next();
	} while (!h->onBoundary);

	return M_PI - sum;
}

// computes the scaling at a wedge
inline double scaling(WedgeCIter w) {
	WedgeCIter next = w->nextWedge();

	Vector a = w->prev()->uv;
	Vector b = w->uv;
	Vector c = next->uv;

	double lij = (b - a).norm();
	double ljk = (c - b).norm();

	double uij = std::log(lij/length(w->halfEdge()->next()->edge()));
	double ujk = std::log(ljk/length(next->halfEdge()->next()->edge()));

	return (lij*uij + ljk*ujk)/(lij + ljk);
}

// computes the tangent at a wedge
inline Vector tangent(WedgeCIter w) {
	Vector a = w->prev()->uv;
	Vector b = w->uv;
	Vector c = w->nextWedge()->uv;

	Vector Tij = b - a;
	Tij.normalize();

	Vector Tjk = c - b;
	Tjk.normalize();

	Vector T = Tij + Tjk;
	T.normalize();

	return T;
}

// returns angle weighted average of adjacent face normals to a vertex
inline Vector normal(VertexCIter v) {
	Vector n;
	HalfEdgeCIter h = v->halfEdge();
	do {
		if (!h->onBoundary) n += normal(h->face(), false)*angle(h->next()->corner());

		h = h->flip()->next();
	} while (h != v->halfEdge());

	n.normalize();
	return n;
}

// returns 2π minus sum of incident angles to a vertex; Note: only valid for interior vertices
inline double angleDefect(VertexCIter v) {
	double sum = 0.0;
	if (v->onBoundary()) return sum;

	HalfEdgeCIter h = v->halfEdge();
	do {
		sum += angle(h->next()->corner());

		h = h->flip()->next();
	} while (h != v->halfEdge());

	return 2*M_PI - sum;
}

// returns the angle in the plane between two line segments
inline double angle(const Vector& a, const Vector& b, const Vector& c) {
	Vector u = b - a;
	u.normalize();

	Vector v = c - b;
	v.normalize();

	double theta = atan2(v.y, v.x) - atan2(u.y, u.x);
	while (theta >= M_PI) theta -= 2*M_PI;
	while (theta < -M_PI) theta += 2*M_PI;

	return theta;
}

// returns face area, computed directly from edge lengths
inline double faceArea(double lij, double ljk, double lki) {
	double s = (lij + ljk + lki)/2.0;
	double arg = s*(s - lij)*(s - ljk)*(s - lki);
	arg = std::max(0.0, arg);
	double area = std::sqrt(arg);

	return area;
}

// returns halfedge cotan weight, computed directly from edge lengths
inline double halfEdgeCotan(double lij, double ljk, double lki) {
	double area = faceArea(lij, ljk, lki);
	double cotan = (-lij*lij + ljk*ljk + lki*lki)/(4.0*area);

	return cotan;
}

// returns corner angle (in radians), computed directly from edge lengths
inline double cornerAngle(double lij, double ljk, double lki) {
	double q = (lij*lij + lki*lki - ljk*ljk)/(2.0*lij*lki);

	return acos(std::max(-1.0, std::min(1.0, q)));
}

} // namespace bff