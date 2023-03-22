#pragma once

#include "bff/linear-algebra/Common.h"

namespace bff {

class SparseMatrix;

class DenseMatrix {
public:
	// constructor
	DenseMatrix(size_t m = 1, size_t n = 1);

	// constructor
	DenseMatrix(cholmod_dense *data);

	// copy constructor
	DenseMatrix(const DenseMatrix& B);

	// assignment operators
	DenseMatrix& operator=(cholmod_dense *data);
	DenseMatrix& operator=(const DenseMatrix& B);

	// destructor
	~DenseMatrix();

	// returns identity
	static DenseMatrix identity(size_t m, size_t n = 1);

	// returns ones
	static DenseMatrix ones(size_t m, size_t n = 1);

	// returns transpose
	DenseMatrix transpose() const;

	// returns number of rows
	size_t nRows() const;

	// returns number of columns
	size_t nCols() const;

	// returns norm. 0: Infinity, 1: 1-norm, 2: 2-norm. Note: 2-norm is only valid for row vectors
	double norm(int norm) const;

	// returns sum
	double sum() const;

	// returns mean
	double mean() const;

	// extracts submatrix in range [r0, r1) x [c0, c1)
	DenseMatrix submatrix(size_t r0, size_t r1, size_t c0 = 0, size_t c1 = 1) const;

	// extracts submatrix with specified row and column indices
	DenseMatrix submatrix(const std::vector<int>& r, const std::vector<int>& c = {0}) const;

	// returns a copy of the cholmod representation
	cholmod_dense* copy() const;

	// returns cholmod representation
	cholmod_dense* toCholmod();

	// math
	friend DenseMatrix operator*(const DenseMatrix& A, double s);
	friend DenseMatrix operator+(const DenseMatrix& A, const DenseMatrix& B);
	friend DenseMatrix operator-(const DenseMatrix& A, const DenseMatrix& B);
	friend DenseMatrix operator*(const DenseMatrix& A, const DenseMatrix& B);
	friend DenseMatrix operator*(const SparseMatrix& A, const DenseMatrix& X);
	friend DenseMatrix operator-(const DenseMatrix& A);

	friend DenseMatrix& operator*=(DenseMatrix& A, double s);
	friend DenseMatrix& operator+=(DenseMatrix& A, const DenseMatrix& B);
	friend DenseMatrix& operator-=(DenseMatrix& A, const DenseMatrix& B);

	// access
	double& operator()(size_t r, size_t c = 0);
	double operator()(size_t r, size_t c = 0) const;

protected:
	// clear
	void clear();

	// member
	cholmod_dense *data;
};

// horizontal concat
DenseMatrix hcat(const DenseMatrix& A, const DenseMatrix& B);

// vertical concat
DenseMatrix vcat(const DenseMatrix& A, const DenseMatrix& B);

} // namespace bff

#include "DenseMatrix.inl"
