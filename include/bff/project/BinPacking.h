#pragma once

#include "bff/mesh/Mesh.h"

namespace bff {

class BinPacking {
public:
	// packs UVs
	static void pack(const Model& model, double scaling,
					 const std::vector<uint8_t>& isSurfaceMappedToSphere,
					 std::vector<Vector>& originalUvIslandCenters,
					 std::vector<Vector>& newUvIslandCenters,
					 std::vector<uint8_t>& isUvIslandFlipped,
					 Vector& modelMinBounds, Vector& modelMaxBounds);
};

} // namespace bff
