#pragma once

#include "bff/mesh/MeshData.h"

namespace bff {

class Distortion {
public:
	// computes quasi conformal error; returns average qc error
	static Vector computeQuasiConformalError(const Model& model);

	// computes area distortion; returns average area distortion
	static Vector computeAreaScaling(const std::vector<Face>& faces);

	// computes area distortion; returns average area distortion
	static Vector computeAreaScaling(const Model& model);

	// returns face color
	static Vector color(FaceCIter f, int meshIndex, bool conformalColors);

private:
	// member
	static std::vector<FaceData<double>> distortion;
};

} // namespace bff
