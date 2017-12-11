#pragma once

#include "Mesh.h"

class Cutter {
public:
    // cut
    static void cut(const vector<VertexIter>& cones, Mesh& mesh);

    // glue
    static void glue(Mesh& mesh);
};
