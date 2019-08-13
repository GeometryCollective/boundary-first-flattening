#include "SparseMatrix.h"

namespace bff {

inline Cholesky::Cholesky(SparseMatrix& A_):
A(A_),
factor(NULL),
validSymbolic(false),
validNumeric(false)
{

}

inline Cholesky::~Cholesky()
{
	clear();
}

inline void Cholesky::clear()
{
	if (factor) {
		cholmod_l_free_factor(&factor, common);
		factor = NULL;
	}

	validSymbolic = false;
	validNumeric = false;
}

inline void Cholesky::clearNumeric()
{
	validNumeric = false;
}

inline void Cholesky::buildSymbolic(cholmod_sparse *C)
{
	clear();

	factor = cholmod_l_analyze(C, common);
	if (factor) validSymbolic = true;
}

inline void Cholesky::buildNumeric(cholmod_sparse *C)
{
	if (factor) validNumeric = (bool)cholmod_l_factorize(C, factor, common);
}

inline void Cholesky::update()
{
	cholmod_sparse *C = A.toCholmod();
	C->stype = 1;

	if (!validSymbolic) buildSymbolic(C);
	if (!validNumeric) buildNumeric(C);
}

inline bool Cholesky::solvePositiveDefinite(DenseMatrix& x, DenseMatrix& b)
{
	update();
	if (factor) x = cholmod_l_solve(CHOLMOD_A, factor, b.toCholmod(), common);

	return common.status() == Common::ErrorCode::ok;
}

} // namespace bff
