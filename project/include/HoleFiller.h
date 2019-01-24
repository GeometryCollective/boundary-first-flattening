#pragma once

#include "Mesh.h"

namespace bff {

class HoleFiller {
public:
	// fills holes. When fillAll is false, all holes except the largest are filled.
	// However, the length of the largest hole must be larger than 50% of the mesh
	// diameter for it not to be filled. Returns true if all holes are filled and
	// false otherwise.
	static bool fill(Mesh& mesh, bool fillAll = false);

private:
	// returns index of longest boundary loop
	static int longestBoundaryLoop(double& loopLength,
								   std::vector<std::vector<HalfEdgeIter>>& boundaryHalfEdges,
								   const std::vector<Face>& boundaries);

	// fill hole
	static void fill(FaceIter b, const std::vector<HalfEdgeIter>& boundaryHalfEdges,
					 Mesh& mesh);
};

} // namespace bff
