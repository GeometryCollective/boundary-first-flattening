#pragma once

#include "DenseMatrix.h"
#include "Cholesky.h"

namespace bff {

class Triplet;

class SparseMatrix {
public:
	// constructor
	SparseMatrix(size_t m = 0, size_t n = 0, size_t nnz = 0);

	// constructor
	SparseMatrix(Triplet& T);

	// constructor
	SparseMatrix(cholmod_sparse *data);

	// copy constructor
	SparseMatrix(const SparseMatrix& B);

	// assignment operators
	SparseMatrix& operator=(cholmod_sparse *data_);
	SparseMatrix& operator=(const SparseMatrix& B);

	// destructor
	~SparseMatrix();

	// returns identity
	static SparseMatrix identity(size_t m, size_t n);

	// return sparse diagonal matrix
	static SparseMatrix diag(const DenseMatrix& d);

	// returns transpose
	SparseMatrix transpose() const;

	// returns number of rows
	size_t nRows() const;

	// returns number of columns
	size_t nCols() const;

	// returns number of non zeros
	size_t nnz() const;

	// returns norm. 0: Infinity, 1: 1-norm
	double norm(int norm) const;

	// extracts submatrix in range [r0, r1) x [c0, c1)
	SparseMatrix submatrix(size_t r0, size_t r1, size_t c0, size_t c1) const;

	// extracts submatrix with specified row and column indices
	SparseMatrix submatrix(const std::vector<int>& r, const std::vector<int>& c) const;

	// returns dense
	DenseMatrix toDense() const;

	// returns a copy of the cholmod representation
	cholmod_sparse* copy() const;

	// returns cholmod representation
	cholmod_sparse* toCholmod();

	// math
	friend SparseMatrix operator*(const SparseMatrix& A, double s);
	friend SparseMatrix operator+(const SparseMatrix& A, const SparseMatrix& B);
	friend SparseMatrix operator-(const SparseMatrix& A, const SparseMatrix& B);
	friend SparseMatrix operator*(const SparseMatrix& A, const SparseMatrix& B);
	friend DenseMatrix operator*(const SparseMatrix& A, const DenseMatrix& X);

	friend SparseMatrix& operator*=(SparseMatrix& A, double s);
	friend SparseMatrix& operator+=(SparseMatrix& A, const SparseMatrix& B);
	friend SparseMatrix& operator-=(SparseMatrix& A, const SparseMatrix& B);

	// member
	Cholesky L;

protected:
	// clear
	void clear();

	// member
	cholmod_sparse *data;
};

class Triplet {
public:
	// constructor
	Triplet(size_t m, size_t n);

	// destructor
	~Triplet();

	// add entry
	void add(size_t i, size_t j, double x);

	// returns choldmod representation
	cholmod_triplet* toCholmod();

protected:
	// increases capacity
	void increaseCapacity();

	// clear
	void clear();

	// member
	cholmod_triplet *data;
	size_t m, n, capacity;
};

} // namespace bff

#include "SparseMatrix.inl"
