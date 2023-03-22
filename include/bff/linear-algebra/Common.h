#pragma once

#include <cholmod.h>

namespace bff  {

// Wraps cholmod_common
class Common {
public:
	// constructor
	Common();

	// destructor
	~Common();

	// error (and warning) codes
	enum class ErrorCode {
		ok,
		methodNotInstalled,
		outOfMemory,
		integerOverflow,
		invalidInput,
		gpuProblem,
		notPositiveDefinite,
		smallDiagonalEntry
	};

	// returns status
	ErrorCode status() const;

	// allows Common to be treated as cholmod_common*
	operator cholmod_common*();

protected:
	// member
	cholmod_common common;
};

} // namespace bff
