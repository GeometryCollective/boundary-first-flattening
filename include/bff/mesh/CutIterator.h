#pragma once

#include "bff/mesh/HalfEdge.h"

namespace bff {

class CutPtrIterator {
public:
	// constructor
	CutPtrIterator(HalfEdgeIter he, bool justStarted);

	// increment, comparison and dereference operators
	const CutPtrIterator& operator++();
	bool operator==(const CutPtrIterator& other) const;
	bool operator!=(const CutPtrIterator& other) const;
	WedgeIter operator*() const;

private:
	// members
	HalfEdgeIter currHe;
	bool justStarted;
};

class CutPtrSet {
public:
	// constructors
	CutPtrSet();
	CutPtrSet(HalfEdgeIter he);

	// begin and end routines
	CutPtrIterator begin();
	CutPtrIterator end();

private:
	// members
	HalfEdgeIter firstHe;
	bool isInvalid;
};

} // namespace bff

#include "CutIterator.inl"
