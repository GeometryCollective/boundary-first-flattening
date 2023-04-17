#pragma once

#include "bff/mesh/Mesh.h"

namespace bff {

class BinPacking {
public:
	// packs UVs
	static void pack(const Model& model,
					 const std::vector<bool>& isSurfaceMappedToSphere,
					 std::vector<Vector>& originalUvIslandCenters,
					 std::vector<Vector>& newUvIslandCenters,
					 std::vector<bool>& isUvIslandFlipped,
					 Vector& modelMinBounds, Vector& modelMaxBounds);
};

} // namespace bff
