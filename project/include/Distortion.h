#pragma once

#include "Types.h"

class Distortion {
public:
    // computes quasi conformal error; returns average qc error
    static Vector computeQuasiConformalError(const vector<Face>& faces);

    // computes area distortion; returns average area distortion
    static Vector computeAreaScaling(const vector<Face>& faces);

    // returns face color
    static Vector color(int faceIndex, bool conformalColors);

private:
    // member
    static vector<double> distortion;
};
