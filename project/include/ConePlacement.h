#pragma once

#include "Bff.h"

namespace bff {

class ConePlacement {
public:
	// error (and warning) codes
	enum class ErrorCode {
		ok,
		factorizationFailed
	};

	// finds S cone singularities and prescibes angles
	static ErrorCode findConesAndPrescribeAngles(int S, std::vector<VertexIter>& cones,
												 DenseMatrix& coneAngles,
												 std::shared_ptr<BFFData> data, Mesh& mesh);

private:
	// initializes the set of cones based on whether the mesh has a boundary
	// or its euler characteristic is non zero
	static int initializeConeSet(VertexData<int>& isCone, Mesh& mesh);

	// collects cone and non-cone indices
	static void separateConeIndices(std::vector<int>& s, std::vector<int>& n,
									const VertexData<int>& isCone,
									const WedgeData<int>& index, const Mesh& mesh,
									bool ignoreBoundary = false);

	// computes target curvatures for the cone set
	static bool computeTargetAngles(DenseMatrix& C, const DenseMatrix& K,
									const SparseMatrix& A,
									const VertexData<int>& isCone,
									const WedgeData<int>& index, const Mesh& mesh);

	// computes target curvatures for the cone set
	static void computeTargetAngles(DenseMatrix& C, const DenseMatrix& u,
									const DenseMatrix& K, const DenseMatrix& k,
									const SparseMatrix& A, const VertexData<int>& isCone,
									const WedgeData<int>& index, Mesh& mesh);

	// computes scale factors
	static bool computeScaleFactors(DenseMatrix& u, const DenseMatrix& K,
									const SparseMatrix& A, const VertexData<int>& isCone,
									const WedgeData<int>& index, const Mesh& mesh);

	// adds cone with largest scale factor
	static bool addConeWithLargestScaleFactor(VertexData<int>& isCone,
											  const DenseMatrix u,
											  const WedgeData<int>& index,
											  const Mesh& mesh);

	// chooses cones using CPMS strategy
	// Section 3.2: http://www.cs.technion.ac.il/~gotsman/AmendedPubl/Miri/EG08_Conf.pdf
	static bool useCpmsStrategy(int S, VertexData<int>& isCone,
								DenseMatrix& C, const DenseMatrix& K,
								SparseMatrix& A, const WedgeData<int>& index,
								Mesh& mesh);

	// chooses cones using CETM strategy
	// Section 5.1: http://multires.caltech.edu/pubs/ConfEquiv.pdf
	static bool useCetmStrategy(int S, VertexData<int>& isCone,
								DenseMatrix& C, const DenseMatrix& K,
								const DenseMatrix& k, const SparseMatrix& A,
								const WedgeData<int>& index, Mesh& mesh);

	// normalizes cone angles to sum to 2pi times euler characteristic
	static void normalizeAngles(DenseMatrix& C, double normalizationFactor);
};

} // namespace bff
