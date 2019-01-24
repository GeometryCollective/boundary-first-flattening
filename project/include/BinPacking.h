#pragma once

#include "Mesh.h"

namespace bff {

class BinPacking {
public:
	// packs uvs
	static void pack(const std::vector<Mesh>& model,
					 const std::vector<bool>& mappedToSphere,
					 std::vector<Vector>& originalCenters,
					 std::vector<Vector>& newCenters,
					 std::vector<bool>& flippedBins,
					 Vector& modelMinBounds, Vector& modelMaxBounds);
};

} // namespace bff
