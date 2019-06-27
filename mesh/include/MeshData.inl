namespace bff {

template<typename Element, std::vector<Element> Mesh::*meshList, typename ElementIter, typename T>
MeshData<Element, meshList, ElementIter, T>::MeshData(const Mesh& mesh_):
data((mesh_.*meshList).size())
{

}

template<typename Element, std::vector<Element> Mesh::*meshList, typename ElementIter, typename T>
MeshData<Element, meshList, ElementIter, T>::MeshData(const Mesh& mesh_, const T& initVal):
data((mesh_.*meshList).size(), initVal)
{

}

template<typename Element, std::vector<Element> Mesh::*meshList, typename ElementIter, typename T>
T& MeshData<Element, meshList, ElementIter, T>::operator[](ElementIter e)
{
	return data[e->index];
}

template<typename Element, std::vector<Element> Mesh::*meshList, typename ElementIter, typename T>
const T& MeshData<Element, meshList, ElementIter, T>::operator[](ElementIter e) const
{
	return data[e->index];
}

template<typename T>
using VertexData = MeshData<Vertex, &Mesh::vertices, VertexCIter, T>;

template<typename T>
using EdgeData = MeshData<Edge, &Mesh::edges, EdgeCIter, T>;

template<typename T>
using FaceData = MeshData<Face, &Mesh::faces, FaceCIter, T>;

template<typename T>
using BoundaryData = MeshData<Face, &Mesh::boundaries, BoundaryCIter, T>;

template<typename T>
using HalfEdgeData = MeshData<HalfEdge, &Mesh::halfEdges, HalfEdgeCIter, T>;

template<typename T>
using CornerData = MeshData<Corner, &Mesh::corners, CornerCIter, T>;

template<typename T>
using WedgeData = MeshData<Wedge, &Mesh::corners, WedgeCIter, T>;

} // namespace bff
