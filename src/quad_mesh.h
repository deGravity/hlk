#pragma once

#include <Eigen/Core>
#include <vector>

namespace hlk {

	struct QuadMesh {
		// Source-of-Truth Matrices
		Eigen::MatrixXd V;
		Eigen::MatrixXi F_q;

		// Derived Structures
		int n; // Number of vertices
		int m; // Number of quads

		// Triangulated Quads (center-fanned)
		// Each triangle represents a side (half-edge) of the quad
		// F_t[:,2] are the center vertices. These correspond to faces
		// V is augmented to have these coordinates
		Eigen::MatrixXi F_t;

		// Edges
		std::vector<int> sides_to_edges;
		Eigen::MatrixXi edges_to_sides;

		// Unique Sides - A single rep chosen for each side
		Eigen::MatrixXi unique_sides;

		// Triangle-Triangle Adjacency in F_t
		// TT[:,0] is side-side adjacency (opposite half-edges)
		// TT[:,1] iterates clockwise through quad sides
		// TT[:,2] iterates counter-clockwise through quad sides
		// -1 if no neighbor
		// TTi is the inverse: it says which side of the neigboring quad
		// was opposite
		// TT[4*q+i,0] / 4 is the i-th quad neighbor of quad q
		Eigen::MatrixX3i TT;
		Eigen::MatrixX3i TTi;

		// Vertex-Triangle Adjacency in F_t
		// VF matches sides to vertices
		// For a quad-mesh-vertex it gives the side adjacencies
		// For a quad-center vertex it gives the sides of the quad
		// There is not a guaranteed order
		// VI gives the index of the vertex in the adjacent triangle
		// for side-to-mesh-vertex matching, this is 0 if outgoing
		// and 1 if incoming - this can be used to filter outgoing and
		// incoming sides. If VI is 2, then the vertex must be a face
		// center
		std::vector<std::vector<int>> VF;
		std::vector<std::vector<int>> VI;

		std::vector<int> valence;

		// is_border_vertex
		// is_boundary_edge
		// is_irregular_vertex
		std::vector<bool> is_boundary_side;
		std::vector<bool> is_singularity;
		std::vector<int> singular_vertices;
		// edges or unique_edge_map

		// boundary information
		std::vector<bool> is_border_vertex;
		Eigen::MatrixX2i boundary_edges; // vertex pairs
		Eigen::VectorXi boundary_sides; // indices
		Eigen::VectorXi boundary_quads; // indices

		// Label Vertices
		// These float epsilon above the real vertices, and are used to create
		// label surfaces
		Eigen::MatrixX3d V_l;

		void populate_derived_fields();

		// Mesh Queries
		int quad(int side); // Get the quad a side belongs to
		int nth_side(int quad, int index);
		int side_u(int side); // src vertex of a side
		int side_v(int side); // dst vertex of a side
		int flip_side(int side); // Get the side opposite between quads, or -1 if a border
		int opposite_side(int side); // Get the side opposite across a quad
		int next_side(int side); // Get the next side (CCW) in a quad
		int prev_side(int side); // Get the prev side (CW) in a quad
		int next_cross_vertex(int side); // Get the next side across vertices if possible
		int prev_cross_vertex(int side); // Get the previous side across vertices if possible
		std::vector<int> out_sides(int vertex); // Get edges adjacent to a vertex with outward orientation
		std::vector<int> in_sides(int vertex); // Get edges adjacent to a vertex with inward orientation
		std::vector<int> sides(int quad); // Get all sides of a quad
		std::vector<int> dual_loop(int side); // Get a quad-dual loop starting from a side
		std::vector<int> dual_loop(int quad, int index); // get a quad-dual look starting from side index of a quad
		std::vector<int> side_loop(int start_side); // trace sides until a singularity or border
		std::vector<int> reverse_side_loop(int start_side); // trace backwards along sides until a singularity or border

	};

}
