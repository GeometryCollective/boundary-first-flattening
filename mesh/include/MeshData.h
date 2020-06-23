#pragma once

#include "Mesh.h"

namespace bff {

template<typename Element, std::vector<Element> Mesh::*meshList, typename ElementIter, typename T>
class MeshData {
public:
	// constructor
	MeshData(const Mesh& mesh);

	// constructor
	MeshData(const Mesh& mesh, const T& initVal);

	// operator[]
	T& operator[](ElementIter e);
	const T& operator[](ElementIter e) const;

private:
	// member
	std::vector<T> data;
};

} // namespace bff

#include "MeshData.inl"
