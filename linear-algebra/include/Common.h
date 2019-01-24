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

	// allows Common to be treated as cholmod_common*
	operator cholmod_common*();

protected:
	// member
	cholmod_common common;
};

} // namespace bff
