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

bool ConePlacement::computeTargetAngles(DenseMatrix& C, const DenseMatrix& K,
										const SparseMatrix& A,
										const VertexData<int>& isCone,
										const WedgeData<int>& index, const Mesh& mesh)
{
	// collect cone and non-cone indices
	std::vector<int> s, n;
	separateConeIndices(s, n, isCone, index, mesh);

	int S = (int)s.size();
	if (S > 0) {
		// initialize cone angles
		DenseMatrix Ks = K.submatrix(s);
		for (int i = 0; i < S; i++) C(s[i]) = Ks(i);

		if (n.size() > 0) {
			// extract submatrices
			SparseMatrix Ann = A.submatrix(n, n);
			SparseMatrix Ans = A.submatrix(n, s);
			DenseMatrix Kn = K.submatrix(n);

			// compute target curvatures
			for (int i = 0; i < S; i++) {
				int I = s[i];

				// solve LGn = δn
				DenseMatrix Gn, Gs(S);
				Gs(i) = 1;
				DenseMatrix delta = -(Ans*Gs);
				if (!Ann.L.solvePositiveDefinite(Gn, delta)) return false;

				// Cs = Ks + Gn^T Kn
				C(I) += (Gn.transpose()*Kn)(0);
			}
		}
	}

	return true;
}

void ConePlacement::computeTargetAngles(DenseMatrix& C, const DenseMatrix& u,
										const DenseMatrix& K, const DenseMatrix& k,
										const SparseMatrix& A, const VertexData<int>& isCone,
										const WedgeData<int>& index, Mesh& mesh)
{
	// collect cone, non-cone and boundary indices indices
	std::vector<int> s, n, b;
	separateConeIndices(s, n, isCone, index, mesh, true);
	for (WedgeCIter w: mesh.cutBoundary()) b.emplace_back(index[w]);

	int I = (int)K.nRows();
	int B = (int)k.nRows();
	int S = (int)s.size();

	if (S > 0) {
		// initialize cone angles
		DenseMatrix Ks = K.submatrix(s);
		for (int i = 0; i < S; i++) C(s[i]) = Ks(i);
		for (int i = 0; i < B; i++) C(b[i]) = k(b[i] - I);

		if (n.size() > 0) {
			// extract submatrices
			SparseMatrix Asn = A.submatrix(s, n);
			SparseMatrix Abn = A.submatrix(b, n);
			DenseMatrix un = u.submatrix(n);

			// compute interior cone angles
			DenseMatrix Cs = -(Asn*un);
			for (int i = 0; i < S; i++) C(s[i]) -= Cs(i);

			// compute boundary cone angles
			DenseMatrix h = -(Abn*un);
			for (int i = 0; i < B; i++) C(b[i]) -= h(i);
		}
	}
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

bool ConePlacement::computeScaleFactors(DenseMatrix& u, const DenseMatrix& K,
										const SparseMatrix& A, const VertexData<int>& isCone,
										const WedgeData<int>& index, const Mesh& mesh)
{
	// collect cone and non-cone indices
	std::vector<int> s, n;
	separateConeIndices(s, n, isCone, index, mesh, true);

	// initialize scale factors
	u = DenseMatrix(K.nRows());

	if (n.size() > 0) {
		// extract submatrices
		DenseMatrix Kn = -K.submatrix(n);
		SparseMatrix Ann = A.submatrix(n, n);

		// compute scale factors
		DenseMatrix un;
		if (!Ann.L.solvePositiveDefinite(un, Kn)) return false;

		// collect scale factors
		for (int i = 0; i < (int)n.size(); i++) {
			u(n[i]) = un(i);
		}
	}

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
	if (!computeTargetAngles(C, K, A, isCone, index, mesh)) return false;

	for (int i = 0; i < S; i++) {
		// compute scale factors
		DenseMatrix u;
		DenseMatrix rhs = -(K - C);
		if (!A.L.solvePositiveDefinite(u, rhs)) return false;

		// add vertex with maximum (abs.) scaling to cone set
		if (!addConeWithLargestScaleFactor(isCone, u, index, mesh)) {
			return false;
		}

		// compute target angles
		if (!computeTargetAngles(C, K, A, isCone, index, mesh)) return false;
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

	// compute scale factors
	DenseMatrix u;
	if (!computeScaleFactors(u, K, A, isCone, index, mesh)) return false;

	for (int i = 0; i < S; i++) {
		// add vertex with maximum (abs.) scaling to cone set
		if (!addConeWithLargestScaleFactor(isCone, u, index, mesh)) {
			return false;
		}

		// compute scale factors
		if (!computeScaleFactors(u, K, A, isCone, index, mesh)) return false;
	}

	// compute cone angles
	computeTargetAngles(C, u, K, k, A, isCone, index, mesh);

	return true;
}

void ConePlacement::normalizeAngles(DenseMatrix& C, double normalizationFactor)
{
	double sum = C.sum();
	if (sum > 1e-8) C *= (normalizationFactor/sum);
}

ConePlacement::ErrorCode ConePlacement::findConesAndPrescribeAngles(
											int S, std::vector<VertexIter>& cones,
											DenseMatrix& coneAngles,
											std::shared_ptr<BFFData> data, Mesh& mesh)
{
	VertexData<int> isCone(mesh, 0);
	DenseMatrix C(data->N);
	DenseMatrix K = vcat(data->K, data->k);
	bool success = true;

	if ((success = useCetmStrategy(S, isCone, C, data->K, data->k, data->A, data->index, mesh))) {
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
	return success ? ConePlacement::ErrorCode::ok :
					 ConePlacement::ErrorCode::factorizationFailed;
}

} // namespace bff
