namespace bff {

template <typename T>
VertexData<T>::VertexData(const Mesh& mesh_):
mesh(mesh_),
data(mesh.vertices.size())
{

}

template <typename T>
VertexData<T>::VertexData(const Mesh& mesh_, const T& initVal):
mesh(mesh_),
data(mesh.vertices.size(), initVal)
{

}

template <typename T>
T& VertexData<T>::operator[](VertexCIter v)
{
	return data[v->index];
}

template <typename T>
const T& VertexData<T>::operator[](VertexCIter v) const
{
	return data[v->index];
}

template <typename T>
EdgeData<T>::EdgeData(const Mesh& mesh_):
mesh(mesh_),
data(mesh.edges.size())
{

}

template <typename T>
EdgeData<T>::EdgeData(const Mesh& mesh_, const T& initVal):
mesh(mesh_),
data(mesh.edges.size(), initVal)
{

}

template <typename T>
T& EdgeData<T>::operator[](EdgeCIter e)
{
	return data[e->index];
}

template <typename T>
const T& EdgeData<T>::operator[](EdgeCIter e) const
{
	return data[e->index];
}

template <typename T>
FaceData<T>::FaceData(const Mesh& mesh_):
mesh(mesh_),
data(mesh.faces.size())
{

}

template <typename T>
FaceData<T>::FaceData(const Mesh& mesh_, const T& initVal):
mesh(mesh_),
data(mesh.faces.size(), initVal)
{

}

template <typename T>
T& FaceData<T>::operator[](FaceCIter f)
{
	return data[f->index];
}

template <typename T>
const T& FaceData<T>::operator[](FaceCIter f) const
{
	return data[f->index];
}

template <typename T>
BoundaryData<T>::BoundaryData(const Mesh& mesh_):
mesh(mesh_),
data(mesh.boundaries.size())
{

}

template <typename T>
BoundaryData<T>::BoundaryData(const Mesh& mesh_, const T& initVal):
mesh(mesh_),
data(mesh.boundaries.size(), initVal)
{

}

template <typename T>
T& BoundaryData<T>::operator[](BoundaryCIter b)
{
	return data[b->index];
}

template <typename T>
const T& BoundaryData<T>::operator[](BoundaryCIter b) const
{
	return data[b->index];
}

template <typename T>
HalfEdgeData<T>::HalfEdgeData(const Mesh& mesh_):
mesh(mesh_),
data(mesh.halfEdges.size())
{

}

template <typename T>
HalfEdgeData<T>::HalfEdgeData(const Mesh& mesh_, const T& initVal):
mesh(mesh_),
data(mesh.halfEdges.size(), initVal)
{

}

template <typename T>
T& HalfEdgeData<T>::operator[](HalfEdgeCIter he)
{
	return data[he->index];
}

template <typename T>
const T& HalfEdgeData<T>::operator[](HalfEdgeCIter he) const
{
	return data[he->index];
}

template <typename T>
CornerData<T>::CornerData(const Mesh& mesh_):
mesh(mesh_),
data(mesh.corners.size())
{

}

template <typename T>
CornerData<T>::CornerData(const Mesh& mesh_, const T& initVal):
mesh(mesh_),
data(mesh.corners.size(), initVal)
{

}

template <typename T>
T& CornerData<T>::operator[](CornerCIter c)
{
	return data[c->index];
}

template <typename T>
const T& CornerData<T>::operator[](CornerCIter c) const
{
	return data[c->index];
}

template <typename T>
WedgeData<T>::WedgeData(const Mesh& mesh_):
mesh(mesh_),
data(mesh.corners.size())
{

}

template <typename T>
WedgeData<T>::WedgeData(const Mesh& mesh_, const T& initVal):
mesh(mesh_),
data(mesh.corners.size(), initVal)
{

}

template <typename T>
T& WedgeData<T>::operator[](WedgeCIter w)
{
	return data[w->index];
}

template <typename T>
const T& WedgeData<T>::operator[](WedgeCIter w) const
{
	return data[w->index];
}

} // namespace bff
