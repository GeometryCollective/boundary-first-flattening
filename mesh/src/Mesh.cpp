#include "Mesh.h"
#include "MeshIO.h"
#include "FaceMesh.h"

Mesh::Mesh()
{

}

Mesh::Mesh(const Mesh& mesh)
{
    *this = mesh;
}

bool Mesh::read(const string& fileName)
{
    static bool firstRead = true;
    ifstream in(fileName.c_str());
    istringstream buffer;
    
    if (firstRead && fileName.empty()) {
        buffer.str(faceMesh);
    
    } else if (!in.is_open()) {
        return false;
    
    } else {
        buffer.str(string(istreambuf_iterator<char>(in), istreambuf_iterator<char>()));
    }

    bool success = false;
    if ((success = MeshIO::read(buffer, *this))) {
        indexElements();
        normalize();
    }
    
    firstRead = false;
    in.close();

    return success;
}

bool Mesh::write(const string& fileName, bool mappedToSphere, bool normalize) const
{
    ofstream out(fileName.c_str());

    if (!out.is_open()) {
        return false;
    }

    MeshIO::write(out, *this, mappedToSphere, normalize);
    out.close();

    return true;
}

int Mesh::eulerCharacteristic() const
{
    return (int)(vertices.size() - edges.size() + faces.size());
}

CutPtrSet Mesh::cutBoundary()
{
    if (boundaries.size() == 0) {
        // if there is no boundary, initialize the iterator with the first edge on the cut
        for (EdgeCIter e = edges.begin(); e != edges.end(); e++) {
            if (e->onCut) return CutPtrSet(e->he);
        }

        return CutPtrSet();
    }

    return CutPtrSet(boundaries[0].he);
}

vector<Wedge>& Mesh::wedges()
{
    return corners;
}

const vector<Wedge>& Mesh::wedges() const
{
    return corners;
}

void Mesh::indexElements()
{
    int index = 0;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->index = index++;
    }

    index = 0;
    for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
        e->index = index++;
    }

    index = 0;
    for (FaceIter f = faces.begin(); f != faces.end(); f++) {
        f->index = index++;
    }

    index = 0;
    for (CornerIter c = corners.begin(); c != corners.end(); c++) {
        c->index = index++;
    }

    index = 0;
    for (HalfEdgeIter h = halfEdges.begin(); h != halfEdges.end(); h++) {
        h->index = index++;
    }

    index = 0;
    for (BoundaryIter b = boundaries.begin(); b != boundaries.end(); b++) {
        b->index = index++;
    }
}

void Mesh::normalize()
{
    // compute center of mass
    Vector cm;
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        cm += v->position;
    }
    cm /= vertices.size();

    // translate to origin and determine radius
    double radius = -1;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position -= cm;
        radius = max(radius, v->position.norm());
    }

    // rescale to unit radius
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position /= radius;
    }
}
