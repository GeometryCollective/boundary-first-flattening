#pragma once

#include <array>
#include <unordered_map>

#include <Vector.h>
#include <MeshIO.h>
#include <BFFMesh.h>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace bff {
	template<typename Traits>
	bff::Mesh openmesh_to_bff_mesh(const OpenMesh::TriMesh_ArrayKernelT<Traits> &mesh) {
		PolygonSoup soup;
		bff::Mesh result;

		for(auto const vh : mesh.vertices()) {
			auto p = mesh.point(vh);
			soup.positions.push_back(bff::Vector(p[0], p[1], p[2]));
		}

		for(auto const fh : mesh.faces()) {
			// We do not handle uncuttable edges, as our mesh definition is for triangles only
			for(auto v_it = mesh.cfv_ccwbegin(fh); v_it != mesh.cfv_ccwend(fh); v_it++) {
				soup.indices.push_back(v_it->idx());
			}
		}

		soup.table.construct(soup.positions.size(), soup.indices);
		std::vector<int> isCuttableEdge(soup.table.getSize(), 1);

		std::string error;
		bff::MeshIO::buildMesh(soup, isCuttableEdge, result, error);
		if(error.size() > 0) {
			std::cerr << "Error converting OpenMesh to BFF-Mesh: " << error << std::endl;
		}

		return result;
	}
	template<typename Traits>
	OpenMesh::TriMesh_ArrayKernelT<Traits> bff_mesh_to_openmesh(bff::Mesh &mesh) {
		typedef OpenMesh::TriMesh_ArrayKernelT<Traits> TriMesh;
		TriMesh result;
		result.request_halfedge_texcoords2D();

		// TODO: assign vertex UVs for inner vertices and halfedge UVs for cuts,
		// so UV shells can be reconstructed.
		// When the cut vertices have face-vertex UVs, the edges from a
		// chart boundary vertex to the inside of the chart will be cut
		// (have different UV vertices) as well.
		// This probably cannot be solved by assigning UVs to vertices and (half)edges
		// but needs a separate UV list. Possibly we should use two functions, one returning
		// the halfedge-UV mesh and one returning a mesh, a UV list and
		// a list of face_vertex->UV_index assignments.

		vector<TriMesh::VertexHandle> vertexHandles;
		map<pair<int, int>, bff::Vector> facevertex_uvs;

		// Create vertices
		for(const auto v: mesh.vertices) {
			// Add all vertices, which are no duplicates
			if(v.referenceIndex == -1) {
				auto p = v.position * (mesh.radius != 0? mesh.radius: 1.0);
				auto vh = result.add_vertex({p[0], p[1], p[2]});
				vertexHandles.push_back(vh);
			}
		}

		// Create UVs
		for(auto w: mesh.wedges()) {
			auto v = w.vertex();
			int vertexIndex = v->referenceIndex == -1 ? v->index : v->referenceIndex;
			int faceIndex = w.face()->index;
			facevertex_uvs[make_pair(faceIndex, vertexIndex)] = w.uv;
		}

		// Create faces
		int uncuttableEdges = 0;
		for(auto f: mesh.faces) {
			if(f.fillsHole) { continue; }
			if(uncuttableEdges > 0) {
				uncuttableEdges--;
				continue;
			}

			HalfEdgeCIter he = f.he;
			while(!he->edge->isCuttable) he = he->next;
			HalfEdgeCIter fhe = he;
			std::unordered_map<int, bool> seenUncuttableEdges;

			std::array<TriMesh::VertexHandle, 3> vhs;
			short j = 0;
			do {
				assert(j < 3);
				VertexCIter v = he->vertex;
				int vIndex = v->referenceIndex == -1 ? v->index : v->referenceIndex;
				vhs[j] = vertexHandles[vIndex];

				he = he->next;
				while(!he->edge->isCuttable) {
					seenUncuttableEdges[he->edge->index] = true;
					he = he->flip->next;
				}
				j++;
			} while(he != fhe);
			uncuttableEdges = (int)seenUncuttableEdges.size();

			result.add_face(vhs.data(), 3);
		}

		// assign UVs
		for(auto heh: result.halfedges()) {
			int f_idx = result.face_handle(heh).idx();
			int v_idx = result.to_vertex_handle(heh).idx();
			bff::Vector uv = facevertex_uvs[make_pair(f_idx, v_idx)];
			result.set_texcoord2D(heh, OpenMesh::Vec2d(uv[0], uv[1]));
		}

		return result;
	}
}
