#include "MeshIO.h"
#include "Mesh.h"
#include <set>
#include <map>
#include <tuple>
#include <iomanip>

class PolygonSoup {
public:
    vector<Vector> positions;
    vector<int> indices;
};

bool MeshIO::read(istringstream& in, Mesh& mesh)
{
    PolygonSoup soup;
    string line;
    
    while (getline(in, line)) {
        stringstream ss(line);
        string token;
        ss >> token;

        if (token == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            soup.positions.push_back(Vector(x, y, z));

        } else if (token == "f") {
            int vertexCount = 0;
            while (ss >> token) {
                stringstream indexStream(token);
                string indexString;
                if (getline(indexStream, indexString, '/')) {
                    int index = stoi(indexString) - 1;
                    soup.indices.push_back(index);

                    vertexCount++;
                }

                if (vertexCount > 3) {
                    cerr << "Only triangle meshes are supported" << endl;
                    return false;
                }
            }
        }
    }

    return buildMesh(soup, mesh);
}

void MeshIO::write(ofstream& out, const Mesh& mesh, bool mappedToSphere, bool normalize)
{
    // write vertices
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        const Vector& p = v->position;
        out << "v " << p.x << " " << p.y << " " << p.z << endl;
    }
    
    double radius = 1.0;
    if (normalize) {
        for (WedgeCIter w = mesh.wedges().begin(); w != mesh.wedges().end(); w++) {
            radius = max(w->uv.norm(), radius);
        }
    }

    // write uvs
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        HalfEdgeCIter he = f->he;
        do {
            const Vector& uv = he->wedge()->uv/radius;
            if (mappedToSphere) {
                double u = 0.5 + atan2(uv.z, uv.x)/(2*M_PI);
                double v = 0.5 - asin(uv.y)/M_PI;
                out << "vt " << u << " " << v << endl;

            } else {
                if (normalize) {
                    double u = (uv.x + 1.0)/2.0;
                    double v = (uv.y + 1.0)/2.0;
                    out << "vt " << u << " " << v << endl;
                    
                } else {
                    out << "vt " << uv.x << " " << uv.y << endl;
                }
            }

            he = he->next;
        } while (he != f->he);
    }

    // write indices
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        out << "f";
        HalfEdgeCIter he = f->he;
        do {
            out << " " << he->next->vertex->index + 1 << "/" << he->index + 1;

            he = he->next;
        } while (he != f->he);
        out << endl;
    }
}

void MeshIO::preallocateElements(const PolygonSoup& soup, Mesh& mesh)
{
    // count the number of edges
    int nBoundaryHalfedges = 0;
    set<pair<int,int>> sortedEdges;
    for (int I = 0; I < (int)soup.indices.size(); I += 3) {
        for (int J = 0; J < 3; J++) {
            int K = (J + 1) % 3;
            int i = soup.indices[I + J];
            int j = soup.indices[I + K];

            if (i > j) swap(i, j);
            pair<int, int> edge(i, j);
            if (sortedEdges.find(edge) != sortedEdges.end()) {
                nBoundaryHalfedges--;

            } else {
                sortedEdges.insert(edge);
                nBoundaryHalfedges++;
            }
        }
    }

    int nVertices = (int)soup.positions.size();
    int nEdges = (int)sortedEdges.size();
    int nFaces = (int)soup.indices.size()/3;
    int nHalfedges = 2*nEdges;
    int nInteriorHalfedges = nHalfedges - nBoundaryHalfedges;

    // clear arrays
    mesh.vertices.clear();
    mesh.edges.clear();
    mesh.faces.clear();
    mesh.corners.clear();
    mesh.halfEdges.clear();
    mesh.boundaries.clear();

    // reserve space
    mesh.vertices.reserve(nVertices);
    mesh.edges.reserve(nEdges);
    mesh.faces.reserve(nFaces);
    mesh.corners.reserve(nInteriorHalfedges);
    mesh.halfEdges.reserve(nHalfedges);
}

vector<HalfEdge> isolated;
bool MeshIO::hasIsolatedVertices(const Mesh& mesh)
{
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (v->isIsolated()) {
            cerr << "Mesh has isolated vertices" << endl;
            return true;
        }
    }

    return false;
}

bool MeshIO::hasIsolatedFaces(const Mesh& mesh)
{
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        int nBoundaryEdges = 0;
        HalfEdgeCIter h = f->he;
        do {
            if (h->flip->onBoundary) nBoundaryEdges++;

            h = h->next;
        } while (h != f->he);

        if (nBoundaryEdges == 3) {
            cerr << "Mesh has isolated faces" << endl;
            return true;
        }
    }

    return false;
}

bool MeshIO::hasNonManifoldVertices(const Mesh& mesh)
{
    map<VertexCIter, int> adjacentFaces;
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        HalfEdgeCIter h = f->he;
        do {
            adjacentFaces[h->vertex]++;

            h = h->next;
        } while (h != f->he);
    }

    for (BoundaryCIter b = mesh.boundaries.begin(); b != mesh.boundaries.end(); b++) {
        HalfEdgeCIter h = b->he;
        do {
            adjacentFaces[h->vertex]++;

            h = h->next;
        } while (h != b->he);
    }

    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (adjacentFaces[v] != v->degree()) {
            return true;
        }
    }

    return false;
}

bool MeshIO::buildMesh(const PolygonSoup& soup, Mesh& mesh)
{
    // preallocate elements
    preallocateElements(soup, mesh);

    // create and insert vertices
    map<int, VertexIter> indexToVertex;
    for (int i = 0; i < (int)soup.positions.size(); i++) {
        VertexIter v = mesh.vertices.insert(mesh.vertices.end(), Vertex());
        v->position = soup.positions[i];
        v->he = isolated.begin();
        indexToVertex[i] = v;
    }

    // create and insert halfedges, edges and "real" faces
    map<pair<int, int>, int> edgeCount;
    map<pair<int, int>, HalfEdgeIter> existingHalfEdges;
    map<HalfEdgeIter, bool> hasFlipEdge;
    for (int I = 0; I < (int)soup.indices.size(); I += 3) {
        // create new face
        FaceIter f = mesh.faces.insert(mesh.faces.end(), Face());

        // create a halfedge for each edge of the newly created face
        vector<HalfEdgeIter> halfEdges(3);
        for (int J = 0; J < 3; J++) {
            halfEdges[J] = mesh.halfEdges.insert(mesh.halfEdges.end(), HalfEdge());
        }

        // initialize the newly created halfedges
        for (int J = 0; J < 3; J++) {
            // current halfedge goes from vertex i to vertex j
            int K = (J + 1) % 3;
            int i = soup.indices[I + J];
            int j = soup.indices[I + K];

            // set the current halfedge's attributes
            HalfEdgeIter h = halfEdges[J];
            h->next = halfEdges[K];
            h->prev = halfEdges[(J + 3 - 1) % 3];
            h->onBoundary = false;
            hasFlipEdge[h] = false;

            // point the new halfedge and vertex i to each other
            VertexIter v = indexToVertex[i];
            h->vertex = v;
            v->he = h;

            // point the new halfedge and face to each other
            h->face = f;
            f->he = h;

            if (i > j) swap(i, j);
            pair<int, int> edge(i, j);
            if (existingHalfEdges.find(edge) != existingHalfEdges.end()) {
                // if a halfedge between vertex i and j has been created in the past, then it
                // is the flip halfedge of the current halfedge
                HalfEdgeIter flip = existingHalfEdges[edge];
                h->flip = flip;
                flip->flip = h;
                h->edge = flip->edge;

                hasFlipEdge[h] = true;
                hasFlipEdge[flip] = true;
                edgeCount[edge]++;

            } else {
                // create an edge and set its halfedge
                EdgeIter e = mesh.edges.insert(mesh.edges.end(), Edge());
                h->edge = e;
                e->he = h;

                // record the newly created edge and halfedge from vertex i to j
                existingHalfEdges[edge] = h;
                edgeCount[edge] = 1;
            }

            // check for non-manifold edges
            if (edgeCount[edge] > 2) {
                cerr << "Mesh has non-manifold edges" << endl;
                return false;
            }
        }
    }

    // create and insert boundary halfedges and "imaginary" faces for boundary cycles
    // also create and insert corners
    HalfEdgeIter end = mesh.halfEdges.end();
    for (HalfEdgeIter h = mesh.halfEdges.begin(); h != end; h++) {
        // if a halfedge has no flip halfedge, create a new face and
        // link it the corresponding boundary cycle
        if (!hasFlipEdge[h]) {
            // create new face
            FaceIter f = mesh.boundaries.insert(mesh.boundaries.end(), Face());

            // walk along boundary cycle
            vector<HalfEdgeIter> boundaryCycle;
            boundaryCycle.reserve(2*mesh.edges.size() - mesh.halfEdges.size());
            HalfEdgeIter he = h;
            do {
                // create a new halfedge
                HalfEdgeIter bH = mesh.halfEdges.insert(mesh.halfEdges.end(), HalfEdge());
                boundaryCycle.push_back(bH);

                // grab the next halfedge along the boundary that does not have a flip halfedge
                HalfEdgeIter nextHe = he->next;
                while (hasFlipEdge[nextHe]) {
                    nextHe = nextHe->flip->next;
                }

                // set the current halfedge's attributes
                bH->vertex = nextHe->vertex;
                bH->edge = he->edge;
                bH->onBoundary = true;

                // point the new halfedge and face to each other
                bH->face = f;
                f->he = bH;

                // point the new halfedge and he to each other
                bH->flip = he;
                he->flip = bH;

                // continue walk
                he = nextHe;
            } while (he != h);

            // link the cycle of boundary halfedges together
            int n = (int)boundaryCycle.size();
            for (int i = 0; i < n; i++) {
                boundaryCycle[i]->next = boundaryCycle[(i + n - 1) % n]; // boundary halfedges are linked in clockwise order
                boundaryCycle[i]->prev = boundaryCycle[(i + 1) % n];
                hasFlipEdge[boundaryCycle[i]] = true;
                hasFlipEdge[boundaryCycle[i]->flip] = true;
            }
        }

        // point the newly created corner and its halfedge to each other
        if (!h->onBoundary) {
            CornerIter c = mesh.corners.insert(mesh.corners.end(), Corner());
            c->he = h;
            h->corner = c;
        }
    }

    // check if mesh has isolated vertices, isolated faces or non-manifold vertices
    if (hasIsolatedVertices(mesh) || hasIsolatedFaces(mesh) || hasNonManifoldVertices(mesh)) {
        return false;
    }

    return true;
}
