#pragma once

#define _USE_MATH_DEFINES

#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include "math.h"
#include "Vector.h"

namespace bff {

class Vertex;
class Edge;
class Face;
class Corner;
class HalfEdge;
class Mesh;

typedef Corner Wedge;
typedef std::vector<Vertex>::iterator            VertexIter;
typedef std::vector<Vertex>::const_iterator      VertexCIter;
typedef std::vector<Edge>::iterator              EdgeIter;
typedef std::vector<Edge>::const_iterator        EdgeCIter;
typedef std::vector<Face>::iterator              FaceIter;
typedef std::vector<Face>::const_iterator        FaceCIter;
typedef std::vector<Corner>::iterator            CornerIter;
typedef std::vector<Corner>::const_iterator      CornerCIter;
typedef std::vector<Corner>::iterator            WedgeIter;
typedef std::vector<Corner>::const_iterator      WedgeCIter;
typedef std::vector<HalfEdge>::iterator          HalfEdgeIter;
typedef std::vector<HalfEdge>::const_iterator    HalfEdgeCIter;
typedef std::vector<Face>::iterator              BoundaryIter;
typedef std::vector<Face>::const_iterator        BoundaryCIter;

} // namespace bff
