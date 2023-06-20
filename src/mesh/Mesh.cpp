#include "bff/mesh/MeshData.h"
#include <limits>

namespace bff {

Mesh::Mesh():
radius(0.0),
cm(0.0, 0.0, 0.0),
status(ErrorCode::ok)
{

}

Mesh::Mesh(const Mesh& mesh):
radius(mesh.radius),
cm(mesh.cm),
status(mesh.status)
{
	// initialize halfEdges
	halfEdges.reserve(mesh.halfEdges.size());
	for (HalfEdgeCIter h = mesh.halfEdges.begin(); h != mesh.halfEdges.end(); h++) {
		HalfEdgeIter hNew = halfEdges.emplace(halfEdges.end(), HalfEdge(*h));
		hNew->setMesh(this);
	}

	// initialize vertices
	vertices.reserve(mesh.vertices.size());
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		VertexIter vNew = vertices.emplace(vertices.end(), Vertex(*v));
		vNew->setMesh(this);
	}

	// initialize edges
	edges.reserve(mesh.edges.size());
	for (EdgeCIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
		EdgeIter eNew = edges.emplace(edges.end(), Edge(*e));
		eNew->setMesh(this);
	}

	// initialize faces
	faces.reserve(mesh.faces.size());
	for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
		FaceIter fNew = faces.emplace(faces.end(), Face(*f));
		fNew->setMesh(this);
	}

	// initialize corners
	corners.reserve(mesh.corners.size());
	for (CornerCIter c = mesh.corners.begin(); c != mesh.corners.end(); c++) {
		CornerIter cNew = corners.emplace(corners.end(), Corner(*c));
		cNew->setMesh(this);
	}

	// initialize boundaries
	boundaries.reserve(mesh.boundaries.size());
	for (BoundaryCIter b = mesh.boundaries.begin(); b != mesh.boundaries.end(); b++) {
		BoundaryIter bNew = boundaries.emplace(boundaries.end(), Face(*b));
		bNew->setMesh(this);
	}
}

int Mesh::eulerCharacteristic() const
{
	return (int)(vertices.size() - edges.size() + faces.size());
}

double Mesh::diameter() const
{
	double maxLimit = std::numeric_limits<double>::max();
	double minLimit = std::numeric_limits<double>::lowest();
	Vector minBounds(maxLimit, maxLimit, maxLimit);
	Vector maxBounds(minLimit, minLimit, minLimit);

	for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
		const Vector& p = v->position;

		minBounds.x = std::min(p.x, minBounds.x);
		minBounds.y = std::min(p.y, minBounds.y);
		minBounds.z = std::min(p.z, minBounds.z);
		maxBounds.x = std::max(p.x, maxBounds.x);
		maxBounds.y = std::max(p.y, maxBounds.y);
		maxBounds.z = std::max(p.z, maxBounds.z);
	}

	return (maxBounds - minBounds).norm();
}

void computeEigenvectors2x2(double a, double b, double c, Vector& v1, Vector& v2)
{
	double disc = std::sqrt((a - c)*(a - c) + 4.0*b*b)/2.0;
	double lambda1 = (a + c)/2.0 - disc;
	double lambda2 = (a + c)/2.0 + disc;

	v1 = b < 0.0 ? Vector(-b, a - lambda1) : Vector(b, lambda2 - a);
	double v1Norm = v1.norm();
	if (v1Norm > 0) v1 /= v1Norm;
	v2 = Vector(-v1.y, v1.x);
}

void Mesh::projectUvsToPcaAxis()
{
	// compute center of mass
	Vector cm;
	int nUvs = 0;
	for (FaceCIter f = faces.begin(); f != faces.end(); f++) {
		if (f->isReal() && !f->fillsHole) {
			Vector centroid = centroidUV(f);

			cm += centroid;
			nUvs++;
		}
	}
	cm /= nUvs;

	// translate UVs to origin
	for (WedgeIter w = wedges().begin(); w != wedges().end(); w++) {
		if (w->isReal()) {
			w->uv -= cm;
		}
	}

	// build covariance matrix
	double a = 0, b = 0, c = 0;
	for (FaceCIter f = faces.begin(); f != faces.end(); f++) {
		if (f->isReal() && !f->fillsHole) {
			Vector centroid = centroidUV(f);

			a += centroid.x*centroid.x;
			b += centroid.x*centroid.y;
			c += centroid.y*centroid.y;
		}
	}

	// compute eigenvectors
	Vector v1, v2;
	computeEigenvectors2x2(a, b, c, v1, v2);

	// project uvs onto principal axes
	for (WedgeIter w = wedges().begin(); w != wedges().end(); w++) {
		if (w->isReal()) {
			Vector& uv = w->uv;
			uv = Vector(dot(v1, uv), dot(v2, uv));
			uv += cm;
		}
	}
}

void Mesh::orientUvsToMinimizeBoundingBox(int nRotations)
{
	// precompute rotations
	double maxRotation = M_PI;
	double minInf = -std::numeric_limits<double>::infinity();
	double maxInf = std::numeric_limits<double>::infinity();
	std::vector<Vector> boxMin(nRotations, Vector(maxInf, maxInf));
	std::vector<Vector> boxMax(nRotations, Vector(minInf, minInf));

	std::vector<Vector> rotations(nRotations);
	for (int i = 0; i < nRotations; i++) {
		double theta = (maxRotation*i)/nRotations;
		rotations[i] = Vector(std::cos(theta), std::sin(theta));
	}

	// try all rotations
	for (WedgeIter w = wedges().begin(); w != wedges().end(); w++) {
		if (w->isReal()) {
			const Vector& uv = w->uv;

			for (int i = 0; i < nRotations; i++) {
				double cosTheta = rotations[i].x;
				double sinTheta = rotations[i].y;
				Vector uvRotated(cosTheta*uv.x - sinTheta*uv.y,
								 sinTheta*uv.x + cosTheta*uv.y);
				boxMin[i].x = std::min(boxMin[i].x, uvRotated.x);
				boxMin[i].y = std::min(boxMin[i].y, uvRotated.y);
				boxMax[i].x = std::max(boxMax[i].x, uvRotated.x);
				boxMax[i].y = std::max(boxMax[i].y, uvRotated.y);
			}
		}
	}

	// find the best rotation
	Vector bestRotation;
	double minScore = std::numeric_limits<double>::infinity();
	for (int i = 0; i < nRotations; i++) {
		Vector extent = boxMax[i] - boxMin[i];
		if (extent.y < minScore) {
			minScore = extent.y;
			bestRotation = rotations[i];
		}
	}

	// apply best rotation
	double cosTheta = bestRotation.x;
	double sinTheta = bestRotation.y;
	for (WedgeIter w = wedges().begin(); w != wedges().end(); w++) {
		if (w->isReal()) {
			const Vector& uv = w->uv;
			Vector uvRotated(cosTheta*uv.x - sinTheta*uv.y,
							 sinTheta*uv.x + cosTheta*uv.y);
			w->uv = uvRotated;
		}
	}
}

double Mesh::areaRatio() const
{
	double totalArea = 0.0;
	double totalAreaUV = 0.0;
	for (FaceCIter f = faces.begin(); f != faces.end(); f++) {
		if (f->isReal() && !f->fillsHole) {
			totalArea += area(f);
			totalAreaUV += areaUV(f);
		}
	}

	if (std::isinf(totalAreaUV) || std::isnan(totalAreaUV)) return 1.0;
	return totalAreaUV > 0.0 ? totalArea/totalAreaUV : 1.0;
}

CutPtrSet Mesh::cutBoundary()
{
	if (boundaries.size() == 0) {
		// if there is no boundary, initialize the iterator with the first edge
		// on the cut
		for (EdgeCIter e = edges.begin(); e != edges.end(); e++) {
			if (e->onCut) return CutPtrSet(e->halfEdge());
		}

		return CutPtrSet();
	}

	return CutPtrSet(boundaries[0].halfEdge());
}

std::vector<Wedge>& Mesh::wedges()
{
	return corners;
}

const std::vector<Wedge>& Mesh::wedges() const
{
	return corners;
}

} // namespace bff
