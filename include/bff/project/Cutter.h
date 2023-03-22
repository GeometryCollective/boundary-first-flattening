#pragma once

#include "bff/mesh/MeshData.h"

namespace bff {

class Cutter {
public:
	// cut
	static void cut(const std::vector<VertexIter>& cones, Mesh& mesh);

	// glue
	static void glue(Mesh& mesh);
};

} // namespace bff
