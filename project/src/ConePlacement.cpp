#include "ConePlacement.h"
#include <limits>

namespace bff {

int ConePlacement::initializeConeSet(VertexData<int>& isCone, Mesh& mesh)
{
	int cones = 0;
	if (mesh.boundaries.size() > 0) {
		// designate all boundary vertices as cone vertices
		for (WedgeCIter w: mesh.cutBoundary()) {
			isCone[w->vertex()] = 1;
			cones++;
		}

	} else if (mesh.eulerCharacteristic() != 0) {
		// surface is closed, select the vertex with largest curvature as a cone
		// singularity if the euler characteristic is greater than 0 and vice versa
		VertexCIter cone;
		double curvature = std::numeric_limits<double>::infinity();
		if (mesh.eulerCharacteristic() > 0) curvature *= -1;

		for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
			double angleDefect = v->angleDefect();
			bool isCandidateCone = mesh.eulerCharacteristic() > 0 ? curvature < angleDefect :
																	curvature > angleDefect;
			if (isCandidateCone) {
				curvature = angleDefect;
				cone = v;
			}
		}

		isCone[cone] = 1;
		cones++;
	}

	return cones;
}

void ConePlacement::separateConeIndices(std::vector<int>& s, std::vector<int>& n,
										const VertexData<int>& isCone,
										const WedgeData<int>& index, const Mesh& mesh,
										bool ignoreBoundary)
{
	// initialize
	s.clear();
	n.clear();

	// collect cone and non-cone indices
	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		int i = index[v->wedge()];

		if (isCone[v]) {
			bool onBoundary = v->onBoundary();
			if (!onBoundary || (onBoundary && !ignoreBoundary)) {
				s.emplace_back(i);
			}

		} else {
			n.emplace_back(i);
		}
	}
}

void ConePlacement::computeTargetAngles(DenseMatrix& C, const DenseMatrix& K,
										const SparseMatrix& A,
										const VertexData<int>& isCone,
										const WedgeData<int>& index, const Mesh& mesh)
{
	// collect cone and non-cone indices
	std::vector<int> s, n;
	separateConeIndices(s, n, isCone, index, mesh);

	int S = (int)s.size();
	if (S > 0) {
		// extract submatrices
		SparseMatrix Ann = A.submatrix(n, n);
		SparseMatrix Ans = A.submatrix(n, s);
		DenseMatrix Kn = K.submatrix(n);
		DenseMatrix Ks = K.submatrix(s);

		// compute target curvatures
		for (int i = 0; i < S; i++) {
			int I = s[i];

			// solve LGn = δn
			DenseMatrix Gn, Gs(S);
			Gs(i) = 1;
			DenseMatrix delta = -(Ans*Gs);
			Ann.L.solvePositiveDefinite(Gn, delta);

			// Cs = Ks + Gn^T Kn
			C(I) = Ks(i);
			if (n.size() > 0) C(I) += (Gn.transpose()*Kn)(0);
		}
	}
}

void ConePlacement::computeTargetAngles(DenseMatrix& C, const DenseMatrix& un,
										const DenseMatrix& K, const DenseMatrix& k,
										const SparseMatrix& A, const std::vector<int>& s,
										const std::vector<int>& n, const std::vector<int>& b)
{
	// initialize
	int I = (int)K.nRows();
	int B = (int)k.nRows();
	int S = (int)s.size();

	// extract submatrices
	SparseMatrix Asn = A.submatrix(s, n);
	SparseMatrix Abn = A.submatrix(b, n);
	DenseMatrix Ks = K.submatrix(s);

	// compute interior cone angles
	DenseMatrix Cs = -(Asn*un);
	for (int i = 0; i < S; i++) {
		C(s[i]) = Ks(i) - Cs(i);
	}

	// compute boundary cone angles
	DenseMatrix h = -(Abn*un);
	for (int i = 0; i < B; i++) {
		C(b[i]) = k(b[i] - I) - h(i);
	}
}

void ConePlacement::computeScaleFactors(DenseMatrix& u, const DenseMatrix& C,
										const DenseMatrix& K, SparseMatrix& A)
{
	DenseMatrix rhs = -(K - C);
	A.L.solvePositiveDefinite(u, rhs);
}

bool ConePlacement::addConeWithLargestScaleFactor(VertexData<int>& isCone,
												  const DenseMatrix u,
												  const WedgeData<int>& index,
												  const Mesh& mesh)
{
	VertexCIter cone;
	double maxU = -std::numeric_limits<double>::infinity();

	for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
		if (!v->onBoundary() && !isCone[v]) {
			int i = index[v->wedge()];

			double absU = std::abs(u(i));
			if (maxU < absU) {
				maxU = absU;
				cone = v;
			}
		}
	}

	if (std::isinf(maxU) || std::isnan(maxU)) {
		for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
			isCone[v] = 0;
		}

		return false;
	}

	isCone[cone] = 1;
	return true;
}

bool ConePlacement::useCpmsStrategy(int S, VertexData<int>& isCone,
									DenseMatrix& C, const DenseMatrix& K,
									SparseMatrix& A, const WedgeData<int>& index,
									Mesh& mesh)
{
	// initialize cone set
	int cones = initializeConeSet(isCone, mesh);
	if (mesh.boundaries.size() == 0) S -= cones;

	// compute target angles
	computeTargetAngles(C, K, A, isCone, index, mesh);

	for (int i = 0; i < S; i++) {
		// compute scale factors
		DenseMatrix u;
		DenseMatrix rhs = -(K - C);
		A.L.solvePositiveDefinite(u, rhs);

		// add vertex with maximum (abs.) scaling to cone set
		if (!addConeWithLargestScaleFactor(isCone, u, index, mesh)) {
			return false;
		}

		// compute target angles
		computeTargetAngles(C, K, A, isCone, index, mesh);
	}

	return true;
}

bool ConePlacement::useCetmStrategy(int S, VertexData<int>& isCone,
									DenseMatrix& C, const DenseMatrix& K,
									const DenseMatrix& k, const SparseMatrix& A,
									const WedgeData<int>& index, Mesh& mesh)
{
	// initialize cone set
	int cones = initializeConeSet(isCone, mesh);
	if (mesh.boundaries.size() == 0) S -= cones;

	// collect cone and non-cone indices
	std::vector<int> s, n, b;
	separateConeIndices(s, n, isCone, index, mesh, true);
	for (WedgeCIter w: mesh.cutBoundary()) b.emplace_back(index[w]);

	// extract submatrices and compute scale factors
	DenseMatrix un;
	DenseMatrix Kn = -K.submatrix(n);
	SparseMatrix Ann = A.submatrix(n, n);
	Ann.L.solvePositiveDefinite(un, Kn);

	for (int i = 0; i < S; i++) {
		// collect scale factors
		DenseMatrix u(K.nRows());
		for (int i = 0; i < (int)n.size(); i++) u(n[i]) = un(i);

		// add vertex with maximum (abs.) scaling to cone set
		if (!addConeWithLargestScaleFactor(isCone, u, index, mesh)) {
			return false;
		}

		// collect cone and non-cone indices
		separateConeIndices(s, n, isCone, index, mesh, true);

		// extract submatrices and compute scale factors
		DenseMatrix Kn = -K.submatrix(n);
		SparseMatrix Ann = A.submatrix(n, n);
		Ann.L.solvePositiveDefinite(un, Kn);
	}

	// compute cone angles
	computeTargetAngles(C, un, K, k, A, s, n, b);

	return true;
}

void ConePlacement::normalizeAngles(DenseMatrix& C, double normalizationFactor)
{
	C *= (normalizationFactor/C.sum());
}

void ConePlacement::findConesAndPrescribeAngles(int S, std::vector<VertexIter>& cones,
												DenseMatrix& coneAngles,
												std::shared_ptr<BFFData> data,
												Mesh& mesh)
{
	VertexData<int> isCone(mesh, 0);
	DenseMatrix C(data->N);
	DenseMatrix K = vcat(data->K, data->k);

	if (useCetmStrategy(S, isCone, C, data->K, data->k, data->A, data->index, mesh)) {
		// set cones
		cones.reserve(S);
		for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
			if (!v->onBoundary() && isCone[v]) {
				cones.emplace_back(v);
			}
		}

		// normalize angles
		normalizeAngles(C, 2*M_PI*mesh.eulerCharacteristic());
	}

	// set cone angles
	coneAngles = C.submatrix(0, data->iN);
}

} // namespace bff
