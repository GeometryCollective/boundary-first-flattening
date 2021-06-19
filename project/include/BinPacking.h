#pragma once

#include "Mesh.h"

namespace bff {

class BinPacking {
public:
	// packs uvs
	static void pack(Model& model, const std::vector<bool>& mappedToSphere,
					 std::vector<Vector>& originalCenters, std::vector<Vector>& newCenters,
					 std::vector<bool>& flippedBins, Vector& modelMinBounds, Vector& modelMaxBounds);
};

} // namespace bff
