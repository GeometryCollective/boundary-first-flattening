#pragma once

#include "bff/linear-algebra/Common.h"

namespace bff {

class DenseMatrix;
class SparseMatrix;

class Cholesky {
public:
	// constructor
	Cholesky(SparseMatrix& A);

	// destructor
	~Cholesky();

	// clears both symbolic and numeric factorization --
	// should be called following any change to nonzero entries
	void clear();

	// clears only numeric factorization --
	// should be called following any change to the values
	// of nonzero entries
	void clearNumeric();

	// solves positive definite
	bool solvePositiveDefinite(DenseMatrix& x, DenseMatrix& b);

protected:
	// builds symbolic factorization
	void buildSymbolic(cholmod_sparse *C);

	// builds numeric factorization
	void buildNumeric(cholmod_sparse *C);

	// updates factorizations
	void update();

	// members
	SparseMatrix& A;
	cholmod_factor *factor;
	bool validSymbolic;
	bool validNumeric;
};

} // namespace bff

#include "Cholesky.inl"
