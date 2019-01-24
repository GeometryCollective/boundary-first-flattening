#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <cblas.h>
#endif

namespace bff {

extern Common common;

inline DenseMatrix::DenseMatrix(size_t m, size_t n)
{
	data = cholmod_l_zeros(m, n, CHOLMOD_REAL, common);
}

inline DenseMatrix::DenseMatrix(cholmod_dense *data_):
data(data_)
{

}

inline DenseMatrix::DenseMatrix(const DenseMatrix& B):
data(B.copy())
{

}

inline DenseMatrix& DenseMatrix::operator=(cholmod_dense *data_)
{
	if (data != data_) {
		clear();
		data = data_;
	}

	return *this;
}

inline DenseMatrix& DenseMatrix::operator=(const DenseMatrix& B)
{
	if (this != &B) {
		clear();
		data = B.copy();
	}

	return *this;
}

inline DenseMatrix::~DenseMatrix()
{
	clear();
}

inline DenseMatrix DenseMatrix::identity(size_t m, size_t n)
{
	return DenseMatrix(cholmod_l_eye(m, n, CHOLMOD_REAL, common));
}

inline DenseMatrix DenseMatrix::ones(size_t m, size_t n)
{
	return DenseMatrix(cholmod_l_ones(m, n, CHOLMOD_REAL, common));
}

inline DenseMatrix DenseMatrix::transpose() const
{
	size_t m = nRows();
	size_t n = nCols();
	DenseMatrix A(n, m);

	for (size_t i = 0; i < m; i++) {
		for (size_t j = 0; j < n; j++) {
			A(j, i) = (*this)(i, j);
		}
	}

	return A;
}

inline size_t DenseMatrix::nRows() const
{
	return data->nrow;
}

inline size_t DenseMatrix::nCols() const
{
	return data->ncol;
}

inline double DenseMatrix::norm(int norm) const
{
	return cholmod_l_norm_dense(data, norm, common);
}

inline double DenseMatrix::sum() const
{
	size_t m = nRows();
	size_t n = nCols();
	double sum = 0.0;

	for (size_t i = 0; i < m; i++) {
		for (size_t j = 0; j < n; j++) {
			sum += (*this)(i, j);
		}
	}

	return sum;
}

inline double DenseMatrix::mean() const
{
	size_t m = nRows();
	size_t n = nCols();

	return sum()/(m*n);
}

inline DenseMatrix DenseMatrix::submatrix(size_t r0, size_t r1, size_t c0, size_t c1) const
{
	size_t m = r1 - r0;
	size_t n = c1 - c0;
	DenseMatrix A(m, n);

	for (size_t i = r0; i < r1; i++) {
		for (size_t j = c0; j < c1; j++) {
			A(i - r0, j - c0) = (*this)(i, j);
		}
	}

	return A;
}

inline DenseMatrix DenseMatrix::submatrix(const std::vector<int>& r,
										  const std::vector<int>& c) const
{
	size_t m = r.size();
	size_t n = c.size();
	DenseMatrix A(m, n);

	for (size_t i = 0; i < m; i++) {
		for (size_t j = 0; j < n; j++) {
			A(i, j) = (*this)(r[i], c[j]);
		}
	}

	return A;
}

inline cholmod_dense* DenseMatrix::copy() const
{
	return cholmod_l_copy_dense(data, common);
}

inline cholmod_dense* DenseMatrix::toCholmod()
{
	return data;
}

inline void scale(double alpha, cholmod_dense *A)
{
	// A = alpha*A
	int N = (int)(A->nrow*A->ncol);
	cblas_dscal(N, alpha, (double *)A->x, 1);
}

inline void add(double alpha, cholmod_dense *A, cholmod_dense *B)
{
	// B = alpha*A + B
	int N = (int)(A->nrow*A->ncol);
	cblas_daxpy(N, alpha, (double *)A->x, 1, (double *)B->x, 1);
}

inline void mul(double alpha, cholmod_dense *A, cholmod_dense *B, double beta, cholmod_dense *C)
{
	// C = alpha*A*B + beta*C
	int M = (int)A->nrow;
	int N = (int)B->ncol;
	int K = (int)B->nrow;
	cblas_dgemm(CblasColMajor, CblasNoTrans, CblasNoTrans, M, N, K,
				alpha, (double *)A->x, M, (double *)B->x, K, beta, (double *)C->x, M);
}

inline DenseMatrix operator*(const DenseMatrix& A, double s)
{
	cholmod_dense *data = A.copy();
	scale(s, data);

	return DenseMatrix(data);
}

inline DenseMatrix operator+(const DenseMatrix& A, const DenseMatrix& B)
{
	DenseMatrix C(B);
	add(1.0, A.data, C.data);

	return C;
}

inline DenseMatrix operator-(const DenseMatrix& A, const DenseMatrix& B)
{
	DenseMatrix C(A);
	add(-1.0, B.data, C.data);

	return C;
}

inline DenseMatrix operator*(const DenseMatrix& A, const DenseMatrix& B)
{
	DenseMatrix C(A.nRows(), B.nCols());
	mul(1.0, A.data, B.data, 0.0, C.data);

	return C;
}

inline DenseMatrix operator-(const DenseMatrix& A)
{
	return A*-1.0;
}

inline DenseMatrix& operator*=(DenseMatrix& A, double s)
{
	scale(s, A.data);

	return A;
}

inline DenseMatrix& operator+=(DenseMatrix& A, const DenseMatrix& B)
{
	add(1.0, B.data, A.data);

	return A;
}

inline DenseMatrix& operator-=(DenseMatrix& A, const DenseMatrix& B)
{
	add(-1.0, B.data, A.data);

	return A;
}

inline double& DenseMatrix::operator()(size_t r, size_t c)
{
	return ((double *)data->x)[c*nRows() + r];
}

inline double DenseMatrix::operator()(size_t r, size_t c) const
{
	return ((double *)data->x)[c*nRows() + r];
}

inline void DenseMatrix::clear()
{
	cholmod_l_free_dense(&data, common);
	data = NULL;
}

inline DenseMatrix hcat(const DenseMatrix& A, const DenseMatrix& B)
{
	size_t m = A.nRows();
	size_t n1 = A.nCols();
	size_t n2 = B.nCols();
	DenseMatrix C(m, n1 + n2);

	for (size_t i = 0; i < m; i++) {
		for (size_t j = 0; j < n1; j++) {
			C(i, j) = A(i, j);
		}

		for (size_t j = 0; j < n2; j++) {
			C(i, n1 + j) = B(i, j);
		}
	}

	return C;
}

inline DenseMatrix vcat(const DenseMatrix& A, const DenseMatrix& B)
{
	size_t m1 = A.nRows();
	size_t m2 = B.nRows();
	size_t n = A.nCols();
	DenseMatrix C(m1 + m2, n);

	for (size_t j = 0; j < n; j++) {
		for (size_t i = 0; i < m1; i++) {
			C(i, j) = A(i, j);
		}

		for (size_t i = 0; i < m2; i++) {
			C(m1 + i, j) = B(i, j);
		}
	}

	return C;
}

} // namespace bff
