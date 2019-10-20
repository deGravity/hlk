#include "quad_mesh.h"

#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/is_border_vertex.h>
#include <igl/is_boundary_edge.h>
#include <igl/boundary_facets.h>

#include <igl/per_vertex_normals.h>
#include <igl/EPS.h>


namespace hlk {
	void QuadMesh::init()
	{
		// Use maxCoeff instead of V.rows() to remain idempotent
		n = F_q.maxCoeff() + 1;
		m = F_q.rows();
		V.conservativeResize(n + m, 3);
		F_t.resize(4 * m, 3);

		// Triangulate quad faces to form F_t
		for (int i = 0; i < m; ++i) {
			V.row(n + i) = Eigen::RowVector3d(0, 0, 0);
			for (int j = 0; j < 4; ++j) {
				V.row(n + i) += V.row(F_q(i, j)) / 4;
				F_t.row(4 * i + j) = Eigen::RowVector3i(F_q(i, j), F_q(i, (j + 1) % 4), n + i);
			}
		}

		// Compute adjacencies on T_t
		igl::triangle_triangle_adjacency(F_t, TT, TTi);
		igl::vertex_triangle_adjacency(V.rows(), F_t, VF, VI);


		igl::boundary_facets(F_t, boundary_edges, boundary_sides, boundary_quads);
		// The quad-centered vertices are after all of the others, so they are n too high
		boundary_quads = boundary_quads.array() - n;

		is_border_vertex = igl::is_border_vertex(F_t);
		is_boundary_side = std::vector<bool>(4*m, false);
		for (int i = 0; i < boundary_sides.size(); ++i) {
			is_boundary_side[boundary_sides[i]] = true;
		}

		// Find compute quad vertex valences
		valence = std::vector<int>(n, 0);
		for (int i = 0; i < F_q.rows(); ++i) {
			for (int j = 0; j < 4; ++j) {
				// Only count non-border sides once
				if (F_q(i, j) < F_q(i, (j + 1) % 4) || flip_side(nth_side(i,j)) < 0) {
					valence[F_q(i, j)] += 1;
					valence[F_q(i, (j + 1) % 4)] += 1;
				}
			}
		}

		// Find singularities
		is_singularity = std::vector<bool>(n, false);
		for (int i = 0; i < n; ++i) {
			if (is_border_vertex[i]) {
				is_singularity[i] = valence[i] > 3;
			}
			else {
				is_singularity[i] = valence[i] != 4;
			}
			if (is_singularity[i]) singular_vertices.push_back(i);
		}


		// Setup floating coordinates for labels
		Eigen::MatrixXd N;
		igl::per_vertex_normals(V, F_t, N);
		V_l = V + N * igl::DOUBLE_EPS * 2;


		// Compute edges (pairs of flip-adjacent sides)
		int num_edges = (4 * m - boundary_sides.size()) / 2;
		e = num_edges;
		sides_to_edges = std::vector<int>(4 * m);
		edges_to_sides.resize(num_edges, 2);

		// Compute unique edge representatives for all sides
		int num_unique_sides =  num_edges + boundary_sides.size();
		unique_sides.resize(num_unique_sides, 2);

		// Populate edges and sides
		int i = 0; // unique side index
		int edge = 0;
		for (int side = 0; side < F_t.rows(); ++side) {
			if (is_boundary_side[side]) {
				unique_sides.row(i++) = F_t.block(side, 0, 1, 2);
			}
			else {
				if (F_t(side, 0) < F_t(side, 1)) {
					unique_sides.row(i++) = F_t.block(side, 0, 1, 2);
					sides_to_edges[side] = edge;
					sides_to_edges[flip_side(side)] = edge;
					edges_to_sides.row(edge) = Eigen::RowVector2i(side, flip_side(side));
					++edge;
				}
			}
		}
		

	}
	int QuadMesh::quad(int side)
	{
		return side / 4;
	}
	int QuadMesh::nth_side(int quad, int index)
	{
		assert(quad < m); // Must be a valid quad index
		return 4*quad+index;
	}
	int QuadMesh::side_u(int side)
	{
		return F_t(side, 0);
		return F_t(side, 1);
	}
	int QuadMesh::side_v(int side)
	{
		return 0;
	}
	int QuadMesh::flip_side(int side)
	{
		return TT(side,0);
	}
	int QuadMesh::opposite_side(int side)
	{
		return next_side(next_side(side));
	}
	int QuadMesh::next_side(int side)
	{
		return TT(side,1);
	}
	int QuadMesh::prev_side(int side)
	{
		return TT(side,2);
	}
	int QuadMesh::next_cross_vertex(int side)
	{
		// Stop at singularities and boundaries
		if (is_singularity[side_v(side)] || flip_side(next_side(side)) < 0) {
			return -1;
		}
		return next_side(flip_side(next_side(side)));
	}
	int QuadMesh::prev_cross_vertex(int side)
	{
		// Stop at singularities and boundaries
		if (is_singularity[side_u(side)] || flip_side(prev_side(side)) < 0) {
			return -1;
		}
		return prev_side(flip_side(prev_side(side)));
	}
	std::vector<int> QuadMesh::out_sides(int vertex)
	{
		assert(vertex < n); // Must be a mesh, not face, vertex
		std::vector<int> outward_sides;
		for (int i = 0; i < VF[vertex].size(); ++i) {
			if (VI[vertex][i] == 0) {
				outward_sides.push_back(VF[vertex][i]);
			}
		}
		return outward_sides;
	}
	std::vector<int> QuadMesh::in_sides(int vertex)
	{
		assert(vertex < n); // Must be a mesh, not face, vertex
		std::vector<int> outward_sides;
		for (int i = 0; i < VF[vertex].size(); ++i) {
			if (VI[vertex][i] == 1) {
				outward_sides.push_back(VF[vertex][i]);
			}
		}
		return outward_sides;
	}
	std::vector<int> QuadMesh::sides(int quad)
	{
		assert(quad < m); // Must be a valid quad index
		// Colud use VF, but don't have to since mesh is pure-quad
		std::vector<int> quad_sides(4);
		for (int i = 0; i < 4; ++i) {
			quad_sides[i] = 4 * quad + i;
		}
		return quad_sides;
	}
	std::vector<int> QuadMesh::dual_loop(int side)
	{
		std::vector<int> quads;
		quads.push_back(quad(side));
		int next = side;
		while (flip_side(next) >= 0) {
			next = opposite_side(flip_side(next));
			if (next == side) break;
			quads.push_back(next);
		}
		next = opposite_side(side);
		while (flip_side(next) >= 0) {
			next = opposite_side(flip_side(next));
			if (next == opposite_side(side)) break;
			quads.push_back(next);
		}

		return quads;
	}
	std::vector<int> QuadMesh::dual_loop(int quad, int index)
	{
		assert(quad < m); // Must be a valid quad index
		return dual_loop(nth_side(quad, index));
	}
	std::vector<int> QuadMesh::side_loop(int start_side)
	{
		std::vector<int> loop;
		loop.push_back(start_side);
		int next_side = next_cross_vertex(start_side);
		while (next_side >= 0 && next_side != start_side) {
			loop.push_back(next_side);
			next_side = next_cross_vertex(next_side);
		}
		return loop;
	}
	std::vector<int> QuadMesh::reverse_side_loop(int start_side)
	{
		std::vector<int> loop;
		loop.push_back(start_side);
		int next_side = prev_cross_vertex(start_side);
		while (next_side >= 0 && next_side != start_side) {
			loop.push_back(next_side);
			next_side = prev_cross_vertex(next_side);
		}
		return loop;
	}
}
