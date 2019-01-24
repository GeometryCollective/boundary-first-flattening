#include <cstring>
using std::memcpy;

namespace bff {

extern Common common;

inline SparseMatrix::SparseMatrix(size_t m, size_t n, size_t nnz):
L(*this)
{
	data = cholmod_l_spzeros(m, n, nnz, CHOLMOD_REAL, common);
}

inline SparseMatrix::SparseMatrix(Triplet& T):
L(*this)
{
	cholmod_triplet *triplet = T.toCholmod();
	data = cholmod_l_triplet_to_sparse(triplet, triplet->nnz, common);
}

inline SparseMatrix::SparseMatrix(cholmod_sparse *data_):
L(*this),
data(data_)
{

}

inline SparseMatrix::SparseMatrix(const SparseMatrix& B):
L(*this),
data(B.copy())
{

}

inline SparseMatrix& SparseMatrix::operator=(cholmod_sparse *data_)
{
	if (data != data_) {
		L.clear();

		clear();
		data = data_;
	}

	return *this;
}

inline SparseMatrix& SparseMatrix::operator=(const SparseMatrix& B)
{
	if (this != &B) {
		L.clear();

		clear();
		data = B.copy();
	}

	return *this;
}

inline SparseMatrix::~SparseMatrix()
{
	clear();
}

inline SparseMatrix SparseMatrix::identity(size_t m, size_t n)
{
	return SparseMatrix(cholmod_l_speye(m, n, CHOLMOD_REAL, common));
}

inline SparseMatrix SparseMatrix::diag(const DenseMatrix& d)
{
	Triplet T(d.nRows(), d.nRows());
	for (size_t i = 0; i < d.nRows(); i++) T.add(i, i, d(i));

	return SparseMatrix(T);
}

inline SparseMatrix SparseMatrix::transpose() const
{
	return SparseMatrix(cholmod_l_transpose(data, 1, common));
}

inline size_t SparseMatrix::nRows() const
{
	return data->nrow;
}

inline size_t SparseMatrix::nCols() const
{
	return data->ncol;
}

inline size_t SparseMatrix::nnz() const
{
	return cholmod_l_nnz(data, common);
}

inline double SparseMatrix::norm(int norm) const
{
	return cholmod_l_norm_sparse(data, norm, common);
}

inline SparseMatrix SparseMatrix::submatrix(size_t r0, size_t r1, size_t c0, size_t c1) const
{
	SuiteSparse_long rsize = (SuiteSparse_long)(r1 - r0);
	SuiteSparse_long *rset = new SuiteSparse_long[rsize];
	for (size_t i = 0; i < rsize; i++) rset[i] = r0 + i;

	SuiteSparse_long csize = (SuiteSparse_long)(c1 - c0);
	SuiteSparse_long *cset = new SuiteSparse_long[csize];
	for (size_t j = 0; j < csize; j++) cset[j] = c0 + j;

	data->stype = 0;
	SparseMatrix A(cholmod_l_submatrix(data, rset, rsize, cset, csize, 1, 1, common));
	delete[] rset;
	delete[] cset;

	return A;
}

inline SparseMatrix SparseMatrix::submatrix(const std::vector<int>& r,
											const std::vector<int>& c) const
{
	SuiteSparse_long rsize = (SuiteSparse_long)r.size();
	SuiteSparse_long *rset = new SuiteSparse_long[rsize];
	for (size_t i = 0; i < rsize; i++) rset[i] = r[i];

	SuiteSparse_long csize = (SuiteSparse_long)c.size();
	SuiteSparse_long *cset = new SuiteSparse_long[csize];
	for (size_t j = 0; j < csize; j++) cset[j] = c[j];

	data->stype = 0;
	SparseMatrix A(cholmod_l_submatrix(data, rset, rsize, cset, csize, 1, 1, common));
	delete[] rset;
	delete[] cset;

	return A;
}

inline DenseMatrix SparseMatrix::toDense() const
{
	return DenseMatrix(cholmod_l_sparse_to_dense(data, common));
}

inline cholmod_sparse* SparseMatrix::copy() const
{
	return cholmod_l_copy_sparse(data, common);
}

inline cholmod_sparse* SparseMatrix::toCholmod()
{
	return data;
}

inline void scale(double s, cholmod_sparse *A)
{
	// A = s*A
	DenseMatrix S(1, 1);
	S(0, 0) = s;
	cholmod_l_scale(S.toCholmod(), CHOLMOD_SCALAR, A, common);
}

inline cholmod_sparse* add(cholmod_sparse *A, cholmod_sparse *B, double alpha[2], double beta[2])
{
	// C = alpha*A + beta*B
	return cholmod_l_add(A, B, alpha, beta, 1, 1, common);
}

inline cholmod_sparse* mul(cholmod_sparse *A, cholmod_sparse *B)
{
	// C = A*B
	return cholmod_l_ssmult(A, B, 0, 1, 1, common);
}

inline void mul(cholmod_sparse *A, cholmod_dense *X, cholmod_dense *Y, double alpha[2], double beta[2])
{
	// Y = alpha*(A*X) + beta*Y
	cholmod_l_sdmult(A, 0, alpha, beta, X, Y, common);
}

inline SparseMatrix operator*(const SparseMatrix& A, double s)
{
	cholmod_sparse *data = A.copy();
	scale(s, data);

	return SparseMatrix(data);
}

inline SparseMatrix operator+(const SparseMatrix& A, const SparseMatrix& B)
{
	double alpha[2] = {1.0, 1.0};
	double beta[2] = {1.0, 1.0};
	return SparseMatrix(add(A.data, B.data, alpha, beta));
}

inline SparseMatrix operator-(const SparseMatrix& A, const SparseMatrix& B)
{
	double alpha[2] = {1.0, 1.0};
	double beta[2] = {-1.0, -1.0};
	return SparseMatrix(add(A.data, B.data, alpha, beta));
}

inline SparseMatrix operator*(const SparseMatrix& A, const SparseMatrix& B)
{
	return SparseMatrix(mul(A.data, B.data));
}

inline DenseMatrix operator*(const SparseMatrix& A, const DenseMatrix& X)
{
	DenseMatrix Y(A.nRows(), X.nCols());
	double alpha[2] = {1.0, 1.0};
	double beta[2] = {0.0, 0.0};
	mul(A.data, X.data, Y.data, alpha, beta);

	return Y;
}

inline SparseMatrix& operator*=(SparseMatrix& A, double s)
{
	scale(s, A.data);
	A.L.clearNumeric();

	return A;
}

inline SparseMatrix& operator+=(SparseMatrix& A, const SparseMatrix& B)
{
	double alpha[2] = {1.0, 1.0};
	double beta[2] = {1.0, 1.0};
	A = add(A.data, B.data, alpha, beta);

	return A;
}

inline SparseMatrix& operator-=(SparseMatrix& A, const SparseMatrix& B)
{
	double alpha[2] = {1.0, 1.0};
	double beta[2] = {-1.0, -1.0};
	A = add(A.data, B.data, alpha, beta);

	return A;
}

inline void SparseMatrix::clear()
{
	cholmod_l_free_sparse(&data, common);
	data = NULL;
}

inline Triplet::Triplet(size_t m_, size_t n_):
m(m_),
n(n_),
capacity(m_)
{
	data = cholmod_l_allocate_triplet(m, n, capacity, 0, CHOLMOD_REAL, common);
	data->nnz = 0;
}

inline Triplet::~Triplet()
{
	clear();
}

inline void Triplet::add(size_t i, size_t j, double x)
{
	if (data->nnz == capacity) increaseCapacity();

	((size_t *)data->i)[data->nnz] = i;
	((size_t *)data->j)[data->nnz] = j;
	((double *)data->x)[data->nnz] = x;
	data->nnz++;
}

inline cholmod_triplet* Triplet::toCholmod()
{
	return data;
}

inline void Triplet::increaseCapacity()
{
	// create triplet with increased capacity
	capacity *= 2;
	cholmod_triplet *newData = cholmod_l_allocate_triplet(m, n, capacity, 0, CHOLMOD_REAL, common);
	memcpy(newData->i, data->i, data->nzmax*sizeof(size_t));
	memcpy(newData->j, data->j, data->nzmax*sizeof(size_t));
	memcpy(newData->x, data->x, data->nzmax*sizeof(double));
	newData->nnz = data->nnz;

	// clear old triplet and assign the newly created triplet to it
	clear();
	data = newData;
}

inline void Triplet::clear()
{
	cholmod_l_free_triplet(&data, common);
	data = NULL;
}

} // namespace bff
