#include "Edge.h"

namespace bff {

inline CutPtrIterator::CutPtrIterator(HalfEdgeIter he, bool justStarted_):
currHe(he),
justStarted(justStarted_)
{

}

inline const CutPtrIterator& CutPtrIterator::operator++()
{
	justStarted = false;
	HalfEdgeIter h = currHe->flip();
	do {

		h = h->prev()->flip(); // loop around one ring counter clockwise
	} while (!h->onBoundary && !h->edge()->onCut);

	currHe = h;
	return *this;
}

inline bool CutPtrIterator::operator==(const CutPtrIterator& other) const
{
	return currHe == other.currHe && justStarted == other.justStarted;
}

inline bool CutPtrIterator::operator!=(const CutPtrIterator& other) const
{
	return !(*this == other);
}

inline WedgeIter CutPtrIterator::operator*() const
{
	return currHe->flip()->prev()->corner();
}

inline CutPtrSet::CutPtrSet():
isInvalid(true)
{

}

inline CutPtrSet::CutPtrSet(HalfEdgeIter he):
firstHe(he),
isInvalid(false)
{

}

inline CutPtrIterator CutPtrSet::begin()
{
	if (isInvalid) return CutPtrIterator(firstHe, false);

	return CutPtrIterator(firstHe, true);
}

inline CutPtrIterator CutPtrSet::end()
{
	return CutPtrIterator(firstHe, false);
}

} // namespace bff
