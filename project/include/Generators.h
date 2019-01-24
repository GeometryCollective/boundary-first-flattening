#pragma once

#include "MeshData.h"

namespace bff {

class Generators {
public:
	// computes generators
	static void compute(Mesh& mesh);

private:
	// builds primal spanning tree
	static void buildPrimalSpanningTree(Mesh& mesh,
										VertexData<VertexCIter>& primalParent);

	// checks whether an edge is in the primal spanning tree
	static bool inPrimalSpanningTree(EdgeCIter e,
									 const VertexData<VertexCIter>& primalParent);

	// builds dual spanning tree
	static void buildDualSpanningTree(Mesh& mesh,
									  FaceData<FaceCIter>& ngon,
									  FaceData<FaceCIter>& dualParent,
									  const VertexData<VertexCIter>& primalParent);

	// checks whether an edge is in the dual spanning tree
	static bool inDualSpanningTree(EdgeCIter e,
								   const FaceData<FaceCIter>& ngon,
								   const FaceData<FaceCIter>& dualParent);

	// returns shared edge between u and v
	static EdgeIter sharedEdge(VertexCIter u, VertexCIter v);

	// creates boundary from generators
	static void createBoundary(Mesh& mesh);
};

} // namespace bff
