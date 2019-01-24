#pragma once

#include "Mesh.h"

namespace bff {

template <typename T>
class VertexData {
public:
	// constructor
	VertexData(const Mesh& mesh);

	// constructor
	VertexData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](VertexCIter v);
	const T& operator[](VertexCIter v) const;

private:
	// members
	const Mesh& mesh;
	std::vector<T> data;
};

template <typename T>
class EdgeData {
public:
	// constructor
	EdgeData(const Mesh& mesh);

	// constructor
	EdgeData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](EdgeCIter e);
	const T& operator[](EdgeCIter e) const;

private:
	// members
	const Mesh& mesh;
	std::vector<T> data;
};

template <typename T>
class FaceData {
public:
	// constructor
	FaceData(const Mesh& mesh);

	// constructor
	FaceData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](FaceCIter f);
	const T& operator[](FaceCIter f) const;

private:
	// members
	const Mesh& mesh;
	std::vector<T> data;
};

template <typename T>
class BoundaryData {
public:
	// constructor
	BoundaryData(const Mesh& mesh);

	// constructor
	BoundaryData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](BoundaryCIter b);
	const T& operator[](BoundaryCIter b) const;

private:
	// members
	const Mesh& mesh;
	std::vector<T> data;
};

template <typename T>
class HalfEdgeData {
public:
	// constructor
	HalfEdgeData(const Mesh& mesh);

	// constructor
	HalfEdgeData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](HalfEdgeCIter he);
	const T& operator[](HalfEdgeCIter he) const;

private:
	// members
	const Mesh& mesh;
	std::vector<T> data;
};

template <typename T>
class CornerData {
public:
	// constructor
	CornerData(const Mesh& mesh);

	// constructor
	CornerData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](CornerCIter c);
	const T& operator[](CornerCIter c) const;

private:
	// members
	const Mesh& mesh;
	std::vector<T> data;
};

template <typename T>
class WedgeData {
public:
	// constructor
	WedgeData(const Mesh& mesh);

	// constructor
	WedgeData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](WedgeCIter w);
	const T& operator[](WedgeCIter w) const;

private:
	// members
	const Mesh& mesh;
	std::vector<T> data;
};

} // namespace bff

#include "MeshData.inl"
