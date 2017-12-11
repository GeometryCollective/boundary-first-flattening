#pragma once

#include "Types.h"
#include "DenseMatrix.h"
#include <fstream>
#include <sstream>

class PolygonSoup;
struct WedgeIterHash;
typedef unordered_map<WedgeCIter, int, WedgeIterHash> WedgeIndex;

class MeshIO {
public:
    // reads data from obj file
    static bool read(istringstream& in, Mesh& mesh);

    // writes data in obj format
    static void write(ofstream& out, const Mesh& mesh, bool mappedToSphere, bool normalize);

private:
    // preallocates mesh elements
    static void preallocateElements(const PolygonSoup& soup, Mesh& mesh);

    // checks if mesh has isolated vertices
    static bool hasIsolatedVertices(const Mesh& mesh);

    // checks if mesh has isolated faces
    static bool hasIsolatedFaces(const Mesh& mesh);

    // checks if mesh has non-manifold vertices
    static bool hasNonManifoldVertices(const Mesh& mesh);

    // builds a halfedge mesh
    static bool buildMesh(const PolygonSoup& soup, Mesh& mesh);
};
