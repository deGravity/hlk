#include "quad_mesh.h"

#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/is_border_vertex.h>
#include <igl/is_boundary_edge.h>
#include <igl/boundary_facets.h>

#include <igl/per_vertex_normals.h>
#include <igl/EPS.h>


namespace hlk {
	void QuadMesh::init() {
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
			} else {
				is_singularity[i] = valence[i] != 4;
			}
			if (is_singularity[i]) singular_vertices.push_back(i);
		}

        // Find faces around singularities
        singular_quads.clear();
        for (int vi : singular_vertices) {
            for (int nrow = 0; nrow < F_q.rows(); ++nrow) {
                for (int j = 0; j < 4; ++j) {
                    if (F_q(nrow, j) == vi) {
                        singular_quads.push_back(nth_side(nrow, 0));
                    }
                }
            }
        }

		// Compute edges (pairs of flip-adjacent sides)
		int num_edges = (4 * m - boundary_sides.size()) / 2;
		e = num_edges;
		sides_to_edges = std::vector<int>(4 * m, -1);
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

	int QuadMesh::quad(int side) {
		return side / 4;
	}
	
    int QuadMesh::nth_side(int quad, int index) {
		assert(quad < m); // Must be a valid quad index
		return 4 * quad + index;
	}

	int QuadMesh::side_u(int side) { return F_t(side, 0); }

	int QuadMesh::side_v(int side) { return F_t(side, 1); }

	int QuadMesh::flip_side(int side) { return TT(side, 0); }

	int QuadMesh::next_side(int side) { return TT(side, 1); }

	int QuadMesh::prev_side(int side) { return TT(side, 2); }

	int QuadMesh::opposite_side(int side) {
		return next_side(next_side(side));
	}
    
    int QuadMesh::next_cross_vertex(int side) {
		// Stop at singularities and boundaries
		if (is_singularity[side_v(side)] || flip_side(next_side(side)) < 0) {
			return -1;
		}
		return next_side(flip_side(next_side(side)));
	}

	int QuadMesh::prev_cross_vertex(int side) {
		// Stop at singularities and boundaries
		if (is_singularity[side_u(side)] || flip_side(prev_side(side)) < 0) {
			return -1;
		}
		return prev_side(flip_side(prev_side(side)));
	}

	std::vector<int> QuadMesh::out_sides(int vertex) {
		assert(vertex < n); // Must be a mesh, not face, vertex
		std::vector<int> outward_sides;
		for (int i = 0; i < VF[vertex].size(); ++i) {
			if (VI[vertex][i] == 0) {
				outward_sides.push_back(VF[vertex][i]);
			}
		}
		return outward_sides;
	}

	std::vector<int> QuadMesh::in_sides(int vertex) {
		assert(vertex < n); // Must be a mesh, not face, vertex
		std::vector<int> outward_sides;
		for (int i = 0; i < VF[vertex].size(); ++i) {
			if (VI[vertex][i] == 1) {
				outward_sides.push_back(VF[vertex][i]);
			}
		}
		return outward_sides;
	}

	std::vector<int> QuadMesh::sides(int quad) {
		assert(quad < m); // Must be a valid quad index
		// Could use VF, but don't have to since mesh is pure-quad
		std::vector<int> quad_sides(4);
		for (int i = 0; i < 4; ++i) {
			quad_sides[i] = 4 * quad + i;
		}
		return quad_sides;
	}

	std::vector<int> QuadMesh::dual_loop(int side) {
		std::vector<int> quads;
		quads.push_back(quad(side));
		int next = side;
		while (flip_side(next) >= 0) {
			next = opposite_side(flip_side(next));
			if (next == side) break;
			quads.push_back(next);
		}
		next = opposite_side(side);
		quads.push_back(next);
		while (flip_side(next) >= 0) {
			next = opposite_side(flip_side(next));
			if (next == opposite_side(side)) break;
			quads.push_back(next);
		}
		return quads;
	}

	std::vector<int> QuadMesh::dual_loop(int quad, int index) {
		assert(quad < m); // Must be a valid quad index
		return dual_loop(nth_side(quad, index));
	}

	std::vector<int> QuadMesh::side_loop(int start_side) {
		std::vector<int> loop;
		loop.push_back(start_side);
		int next_side = next_cross_vertex(start_side);
		while (next_side >= 0 && next_side != start_side) {
			loop.push_back(next_side);
			next_side = next_cross_vertex(next_side);
		}
		return loop;
	}

	std::vector<int> QuadMesh::reverse_side_loop(int start_side) {
		std::vector<int> loop;
		loop.push_back(start_side);
		int next_side = prev_cross_vertex(start_side);
		while (next_side >= 0 && next_side != start_side) {
			loop.push_back(next_side);
			next_side = prev_cross_vertex(next_side);
		}
		return loop;
	}


    bool QuadMesh::is_course_loop(int curr_he, Cardinal c) {

        int count = 0;
        int max_to_check = m; // disjoint_set.max_row_length;
        std::map<int, Cardinal> face_directions;
        std::unordered_set<int> visited_hes;

        while (count < max_to_check) {
            int quad_face = quad(curr_he);
            std::vector<int> hes = sides(quad_face);
            for (int idx = 0; idx < 4; ++idx) {
                visited_hes.emplace(hes[idx]);
                face_directions[hes[idx]] = (Cardinal)((c + idx) % 4);
            }
            int opp_he = opposite_side(curr_he);
            if (flip_side(opp_he) < 0 || is_seam_edge[opp_he]) break;
            curr_he = flip_side(opp_he);
            if (visited_hes.find(curr_he) != visited_hes.end()) {
                return face_directions[curr_he] == c;
            }
            ++count;
        }

        return false;
    }

    bool QuadMesh::perp_direction_check(
        int curr_he,
        std::unordered_set<int>& ortho_visited,
        const std::map<int, Cardinal>& face_directions) {

        Cardinal c_opp = (Cardinal)((face_directions.at(curr_he) + 2) % 4);

        while (true) {

            int quad_face = quad(curr_he);
            std::vector<int> hes = sides(quad_face);
            for (int idx = 0; idx < 4; ++idx) {
                ortho_visited.emplace(hes[idx]);
            }
            int opp_he = opposite_side(curr_he);
            if (flip_side(opp_he) < 0 || is_seam_edge[opp_he]) break;

            curr_he = flip_side(opp_he);
            if (ortho_visited.find(curr_he) != ortho_visited.end()) break;

            if (face_directions.find(curr_he) != face_directions.end()) {
                if (face_directions.at(curr_he) == c_opp) {
                    // reached back, this is a helix
                    return true;
                }
            }
        }

        return false;
    }

    bool QuadMesh::helix_free(std::unordered_set<int>& helix, Cardinal c) {

        int nskip = 1; // disjoint_set.min_row_length;
        std::unordered_set<int> all_helices;
        helix.clear();

        for (int curr_he : singular_quads) {

            if (is_course_loop(curr_he, c)) continue;

            std::map<int, Cardinal> face_directions;
            std::unordered_set<int> visited;
            bool found = false;
            int count = 0;

            while (true) {
                int quad_face = quad(curr_he);
                std::vector<int> hes = sides(quad_face);
                for (int idx = 0; idx < 4; ++idx) {
                    visited.emplace(hes[idx]);
                    face_directions[hes[idx]] = (Cardinal)((c + idx) % 4);
                }
                int opp_he = opposite_side(curr_he);
                if (flip_side(opp_he) < 0 || is_seam_edge[opp_he]) break;

                if (count > nskip) {
                    std::unordered_set<int> ortho_visited;
                    found = perp_direction_check(prev_side(curr_he), ortho_visited, face_directions);
                    if (!found) {
                        ortho_visited.clear();
                        found = perp_direction_check(prev_side(opp_he), ortho_visited, face_directions);
                    }
                }

                if (found) break;

                curr_he = flip_side(opp_he);
                ++count;
                if (visited.find(curr_he) != visited.end()) break;
            }

            if (found) {
                std::unordered_set<int> curr_helix;
                int he = curr_he;
                while (true) {
                    int quad_face = quad(he);
                    for (const int he : sides(quad_face)) {
                        curr_helix.emplace(he);
                    }
                    he = opposite_side(he);
                    if (flip_side(he) < 0) break;
                    he = flip_side(he);
                    if (he == curr_he) break;
                }
                if (helix.size() < curr_helix.size()) {
                    helix = curr_helix;
                    all_helices.insert(curr_helix.begin(), curr_helix.end());
                }
            }
        }

        return all_helices.empty();
    }

}
