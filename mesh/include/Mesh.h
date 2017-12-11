#pragma once

#include "HalfEdge.h"
#include "CutIterator.h"

class DenseMatrix;
typedef unordered_map<WedgeCIter, int, WedgeIterHash> WedgeIndex;

class Mesh {
public:
    // constructor
    Mesh();

    // copy constructor
    Mesh(const Mesh& mesh);

    // reads mesh from file
    bool read(const string& fileName);

    // writes mesh to file
    bool write(const string& fileName, bool mappedToSphere, bool normalize) const;

    // returns euler characteristic
    int eulerCharacteristic() const;

    // range based for loop over the cut boundary
    CutPtrSet cutBoundary(); // valid only for 1 boundary loop, by default, this is the first loop

    // returns reference to wedges (a.k.a. corners)
    vector<Wedge>& wedges();
    const vector<Wedge>& wedges() const;

    // member variables
    vector<Vertex> vertices;
    vector<Edge> edges;
    vector<Face> faces;
    vector<Corner> corners;
    vector<HalfEdge> halfEdges;
    vector<Face> boundaries;

protected:
    // assigns indices to mesh elements
    void indexElements();

    // centers mesh around origin and scales to unit radius
    void normalize();
};
