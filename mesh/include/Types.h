#pragma once

#define _USE_MATH_DEFINES

#include <stdlib.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include "math.h"
#include "Vector.h"

using namespace std;

class Vertex;
class Edge;
class Face;
class Corner;
class HalfEdge;
class Mesh;

typedef Corner Wedge;
typedef vector<Vertex>::iterator            VertexIter;
typedef vector<Vertex>::const_iterator      VertexCIter;
typedef vector<Edge>::iterator              EdgeIter;
typedef vector<Edge>::const_iterator        EdgeCIter;
typedef vector<Face>::iterator              FaceIter;
typedef vector<Face>::const_iterator        FaceCIter;
typedef vector<Corner>::iterator            CornerIter;
typedef vector<Corner>::const_iterator      CornerCIter;
typedef vector<Corner>::iterator            WedgeIter;
typedef vector<Corner>::const_iterator      WedgeCIter;
typedef vector<HalfEdge>::iterator          HalfEdgeIter;
typedef vector<HalfEdge>::const_iterator    HalfEdgeCIter;
typedef vector<Face>::iterator              BoundaryIter;
typedef vector<Face>::const_iterator        BoundaryCIter;
