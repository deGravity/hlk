#include <directional/representative_to_raw.h>
#include <directional/polyvector_field.h>
#include <directional/polyvector_to_raw.h>
#include <igl/AABB.h>
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/copyleft/comiso/nrosy.h>
#include <igl/local_basis.h>
#include <igl/principal_curvature.h>
#include <igl/rotate_vectors.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/triangle_triangle_adjacency.h>

#include "meshing_algorithms.h"
#include "remeshing_plugin.h"

namespace hlk {

void RemeshingMenu::reset_face_vectors() {
    face_vectors.clear();
    for (int i = 0; i < F.rows(); i++) {
        FaceVector fv;
        fv.face_id = i;
        Eigen::Vector3d v0 = V.row(F.row(i)[0]);
        Eigen::Vector3d v1 = V.row(F.row(i)[1]);
        Eigen::Vector3d v2 = V.row(F.row(i)[2]);
        fv.center = (v0 + v1 + v2) / 3.0;
        fv.normal = (v1 - v0).cross(v2 - v0).normalized();
        fv.assigned = { false, false };
        fv.frame = { Eigen::Vector3d(), Eigen::Vector3d() };
        face_vectors.push_back(fv);
    }
}

void RemeshingMenu::reset_field() {
    reset_face_vectors();
    split_edges.clear();
    setup_boundary();
    interpolate_field();
}

void RemeshingMenu::init_curvature_field() {
    Eigen::MatrixXd PD1, PD2; // unit directions per vertex
    Eigen::VectorXd PV1, PV2; // magnitude
    igl::principal_curvature(V, F, PD1, PD2, PV1, PV2);

    Eigen::MatrixXd FD1, FD2; // directions per face
    FD1.resize(F.rows(), 3); FD1.setZero();
    FD2.resize(F.rows(), 3); FD2.setZero();
    for (int i = 0; i < F.rows(); ++i)
        for (int j = 0; j < 3; ++j) {
            FD1.row(i) += PV1(F(i, j)) * PD1.row(F(i, j));
            FD2.row(i) += PV2(F(i, j)) * PD2.row(F(i, j));
        }
    FD1.array() /= 3;
    FD2.array() /= 3;
    direction_field[0] = FD1;
    direction_field[1] = FD2;

    update_vectors_from_field(0);
    update_vectors_from_field(1);
}

void RemeshingMenu::setup_boundary() {
	std::vector<std::vector<int>> indices;
	igl::boundary_loop(F, indices);

	int top_index = 0;
	double maximal_y = -100000.0;
	for (int i = 0; i < indices.size(); i++) {
		Eigen::Vector3d center(0.0, 0.0, 0.0);
		for (int j = 0; j < indices[i].size(); j++) {
			center += V.row(indices[i][j]);
		}
		center /= indices[i].size();
		if (maximal_y < center[1]) {
			maximal_y = center[1];
			top_index = i;
		}
	}

	for (int i = 0; i < F.rows(); i++) {
		int index_0 = F.row(i)[0];
		int index_1 = F.row(i)[1];
		int index_2 = F.row(i)[2];
		int b0 = row_index_of(indices, index_0);
		int b1 = row_index_of(indices, index_1);
		int b2 = row_index_of(indices, index_2);

		int index_00;
		int index_11;
		int index_22;
		bool goon = false;
		bool opposite = false;
		if (b0 >= 0 && b1 >= 0) { // NOTE: maybe these can be connected with else
			index_00 = index_0;
			index_11 = index_1;
			index_22 = index_2;
			goon = true;
			if (b0 == top_index) opposite = true;
		}
		if (b1 >= 0 && b2 >= 0) {
			index_00 = index_1;
			index_11 = index_2;
			index_22 = index_0;
			goon = true;
			if (b1 == top_index) opposite = true;
		}
		if (b2 >= 0 && b0 >= 0) {
			index_00 = index_2;
			index_11 = index_0;
			index_22 = index_1;
			goon = true;
			if (b2 == top_index) opposite = true;
		}

		if (goon) {
			Eigen::Vector3d edge = V.row(index_00) - V.row(index_11);
			Eigen::Vector3d v1 = face_vectors[i].normal.cross(edge);
			Eigen::Vector3d center = (V.row(index_00) + V.row(index_11)) / 2.0;
			Eigen::Vector3d v2 = (Eigen::Vector3d) V.row(index_22) - center;

			double angle = angle_between(v1, v2);
			if (angle > M_PI / 2.0) v1 = -v1;
			if (opposite) v1 = -v1;

            face_vectors[i].frame[1] = v1; // still set to be wale
			face_vectors[i].assigned[1] = true;
            // initialize the course direction to be perpendicular
            face_vectors[i].frame[0] = face_vectors[i].frame[1].cross(face_vectors[i].normal);
            face_vectors[i].assigned[0] = true;
        }
	}
}

void RemeshingMenu::interpolate_cross_field(Eigen::VectorXd& S, int direction) {
    // Set up cross field constraints.
    int hard_constraint_count = 0;
    int soft_constraint_count = 0;
    for (auto& face_vector : face_vectors) {
        if (face_vector.assigned[direction]) {
            if (face_vector.is_hard) {
                ++hard_constraint_count;
            } else {
                ++soft_constraint_count;
            }
        }
    }

    Eigen::VectorXi hard_constraint_indices(hard_constraint_count);
    Eigen::MatrixXd hard_constraints(hard_constraint_count, 3);
    Eigen::VectorXi soft_constraint_indices(soft_constraint_count);
    Eigen::VectorXd soft_constraint_weights(soft_constraint_count);
    Eigen::MatrixXd soft_constraints(soft_constraint_count, 3);
    c_b.resize(hard_constraint_count + soft_constraint_count); c_b.setZero();
    c_bc.resize(hard_constraint_count + soft_constraint_count, 6); c_bc.setZero();
    c_blevel.resize(hard_constraint_count + soft_constraint_count); c_blevel.setZero();

    int idx_hard = 0;
    int idx_soft = 0;
    int idx = 0;
    for (auto& face_vector : face_vectors) {
        if (face_vector.assigned[direction]) {
            if (face_vector.is_hard) {
                hard_constraint_indices[idx_hard] = face_vector.face_id;
                hard_constraints.row(idx_hard) = face_vector.frame[direction].normalized();
                ++idx_hard;
            } else {
                soft_constraint_indices[idx_soft] = face_vector.face_id;
                soft_constraint_weights[idx_soft] = 1.0;
                soft_constraints.row(idx_soft) = face_vector.frame[direction].normalized();
                ++idx_soft;
            }
            c_b(idx) = face_vector.face_id;
            c_bc.block<1, 3>(idx, 0) = face_vector.frame[direction];
            c_blevel(idx) = 1;
            if (face_vector.assigned[1-direction]) {
                c_bc.block<1, 3>(idx, 3) = face_vector.frame[1-direction];
                c_blevel(idx) = 2;
            }
            ++idx;
        }
    }

    igl::copyleft::comiso::nrosy(
        V, F,
        hard_constraint_indices, hard_constraints,
        soft_constraint_indices, soft_constraint_weights, soft_constraints,
        rosy, soft_constraint_strength, direction_field[direction], S);
}

void RemeshingMenu::update_vectors_from_field(int direction) {
    if (symmetrize_nrosy) {
        bool axes[3];
        axes[0] = symmetry_mode_yz;
        axes[1] = symmetry_mode_xz;
        axes[2] = symmetry_mode_xy;
        symmetrizer.symmetrize(direction_field[direction], axes);
    }

    // Populate face vectors.
    const Eigen::MatrixXd& PD1 = direction_field[direction];
    for (int i = 0; i < F.rows(); ++i) {
        double x = PD1.row(i) * B1.row(i).transpose();
        double y = PD1.row(i) * B2.row(i).transpose();
        double angle = atan2(y, x);
        face_vectors[i].frame[direction] = cos(angle) * B1.row(i) + sin(angle) * B2.row(i);
        if (direction == 1) {
            face_vectors[i].base_vector = cos(angle + igl::PI / 2.0) * B1.row(i) + sin(angle + igl::PI / 2.0) * B2.row(i);
        }
    }
}

void RemeshingMenu::interpolate_field() {
    Eigen::VectorXd S;

    if (miq_mode == MIQMode::POLYVECTOR) {
        // Interpolate both directions separately first.
        interpolate_cross_field(S, 0);
        update_vectors_from_field(0);
        interpolate_cross_field(S, 1);
        update_vectors_from_field(1);

        // Set up constraints.
        std::vector<int> constrained_faces;
        std::vector<int> wale_constrained_faces;
        std::vector<std::vector<Eigen::Vector3d>> constraints;
        std::vector<std::vector<Eigen::Vector3d>> wale_constraints;
        for (FaceVector& fv : face_vectors) {
            if (fv.assigned[0] || fv.assigned[1]) {
                // for polyvector field interpolation
                constrained_faces.push_back(fv.face_id);
                std::vector<Eigen::Vector3d> face_constraints;
                if (fv.assigned[0] && fv.assigned[1]) {
                    face_constraints = { fv.frame[1], fv.frame[0], -fv.frame[1], -fv.frame[0] };
                } else {
                    if (fv.assigned[0]) {
                        Eigen::Vector3d ortho_dir = fv.normal.cross(fv.frame[0]);
                        face_constraints = { fv.frame[0], ortho_dir, -fv.frame[0], -ortho_dir };
                    } else { // fv.assigned[1]
                        Eigen::Vector3d ortho_dir = fv.normal.cross(fv.frame[1]);
                        face_constraints = { ortho_dir, fv.frame[1], -ortho_dir, -fv.frame[1] };
                    }
                }
                constraints.push_back(face_constraints);
                // for curl reduction precomputation
                if (fv.assigned[1]) {
                    wale_constrained_faces.push_back(fv.face_id);
                    std::vector<Eigen::Vector3d> wale_face_constraints;
                    if (fv.assigned[0]) {
                        wale_face_constraints = { fv.frame[1], fv.frame[0] };
                    } else {
                        wale_face_constraints = { fv.frame[1] };
                    }
                    wale_constraints.push_back(wale_face_constraints);
                }
            }
        }

        p_b.resize(constrained_faces.size()); p_b.setZero();
        p_bc.resize(constrained_faces.size(), 3 * rosy); p_bc.setZero();
        for (int i = 0; i < constrained_faces.size(); ++i) {
            p_b(i) = constrained_faces[i];
            for (int j = 0; j < rosy; ++j) {
                p_bc.block<1, 3>(i, 3 * j) = constraints[i][j];
            }
        }

        c_b.resize(wale_constrained_faces.size()); c_b.setZero();
        c_bc.resize(wale_constrained_faces.size(), 6); c_bc.setZero();
        c_blevel.resize(wale_constrained_faces.size()); c_blevel.setZero();
        for (int i = 0; i < wale_constrained_faces.size(); ++i) {
            c_b(i) = wale_constrained_faces[i];
            c_blevel(i) = wale_constraints[i].size();
            c_bc.block<1, 3>(i, 0) = wale_constraints[i][0];
            if (c_blevel(i) == 2) {
                c_bc.block<1, 3>(i, 3) = wale_constraints[i][1];
            }
        }

        directional::polyvector_field(V, F, p_b, p_bc, rosy, polyvector_field);
        directional::polyvector_to_raw(V, F, polyvector_field, rosy, rawField);

        viewing_mode = ViewingMode::MESH_FIELD;

    } else { // miq_mode == MIQMode::CROSS

        interpolate_cross_field(S);
        update_vectors_from_field();
        directional::representative_to_raw(V, F, direction_field[1], rosy, rawField);

        int s_count = 0;
        for (int i = 0; i < S.rows(); ++i) {
            s_count += S(i) > 0.01 ? 1 : 0;
        }
        std::cout << "Singularity Count = " << s_count << "\n";

        viewing_mode = ViewingMode::MESH_ONLY;
    }

    has_direction_field = true;
    has_integer_grid = false;
    has_curl = false;
    update_visualization();
}

void RemeshingMenu::generate_integer_grid() {

    if (miq_mode == MIQMode::POLYVECTOR) {
        Meshing::polyvector_parametrize(
            V, F, rosy, EV, EF, FE,
            rawField, combedField,
            combedMatching, combedEffort,
            singVertices, singIndices,
            VMeshCut, FMeshCut, cutUV,
            1. / gradient_size, isInteger
        );

    } else {
        std::vector<std::vector<int>> hard_edges;

        Eigen::MatrixXi TT, TTi;
        igl::triangle_triangle_adjacency(F, TT, TTi);
        for (int f = 0; f < F.rows(); ++f) {
            for (int i = 0; i < 3; ++i) {
                if (TT(f, i) == -1) {
                    hard_edges.push_back({ f,i });
                }
            }
        }

        std::vector<std::vector<int>> VF, VI;
        igl::vertex_triangle_adjacency(V.rows(), F, VF, VI);
        for (auto& split_edge : split_edges) {
            auto v0 = split_edge.index_0;
            auto v1 = split_edge.index_1;
            for (int i = 0; i < VF[v0].size(); ++i) {
                int f = VF[v0][i];
                int idx = VI[v0][i];
                assert(F(f, idx) == v0);
                if (F(f, (idx + 1) % 3) == v1) {
                    hard_edges.push_back({ f, idx });
                    continue;
                }
            }
        }

        Meshing::cross_field_miq(
            direction_field[1],
            V,
            F,
            hard_edges,
            gradient_size,
            stiffen_iter,
            V_uv,
            F_uv
        );
    }
    
    has_integer_grid = true;
}

void RemeshingMenu::init_curl() {
    Meshing::init_curl(
        V, F, rosy, EV, EF, FE,
        c_b, c_bc, c_blevel,
        rawField, combedField,
        combedMatching, combedEffort,
        curl, singVertices, singIndices,
        AE2F, curlMax, curlMaxOrig);
    has_curl = true;
}

void RemeshingMenu::reduce_curl() {
    Meshing::reduce_curl(
        V, F, rosy, EV, EF, FE,
        rawField, combedField,
        combedMatching, combedEffort,
        curl, singVertices, singIndices,
        curlMax);
    if (miq_mode == MIQMode::CROSS) {
        direction_field[1] = combedField.block(0, 0, F.rows(), 3);
    }
}

void RemeshingMenu::init_quad_seams() {
    if (!is_quad_meshed) { return; }

    igl::AABB<Eigen::MatrixXd, 3> aabb_tree;
    aabb_tree.init(quad_mesh.V, quad_mesh.F_t);

    quad_mesh.is_seam_edge.clear();
    quad_mesh.is_seam_edge = std::vector<bool>(quad_mesh.e, false);
    for (const SplitEdge& se : split_edges) {
        Eigen::Vector3d v0 = V.row(se.index_0);
        Eigen::Vector3d v1 = V.row(se.index_1);
        Eigen::Vector3d edge = v1 - v0;
        Eigen::Vector3d nudge_dir = edge.cross(se.normal);

        Eigen::Vector3d nudged_midpoint = (v0 + v1) / 2. + 0.001 * mesh_size * nudge_dir;
        int fid;
        Eigen::RowVector3d C;
        aabb_tree.squared_distance(quad_mesh.V, quad_mesh.F_t, nudged_midpoint, fid, C);
        quad_mesh.is_seam_edge[fid] = true;
        if (quad_mesh.flip_side(fid) > 0) {
            quad_mesh.is_seam_edge[quad_mesh.flip_side(fid)] = true;
        }
    }
}

void RemeshingMenu::quad_helix_finding() {
    if (!is_quad_meshed) { return; }

    std::unordered_set<int> longest_helix;
    if (!quad_mesh.helix_free(longest_helix, cardinal)) {
        std::cout << "[remeshing] there exists a helix somewhere... highlighted in green.\n";
        Eigen::MatrixXd interactive_colors(quad_mesh.m * 4, 3);
        interactive_colors.setOnes();
        for (auto iter = longest_helix.begin(); iter != longest_helix.end(); ++iter) {
            interactive_colors.row((*iter)) = Eigen::RowVector3d(0.8, 1., 0.6);
        }
        stylize_quad_mesh(interactive_colors);
    } else {
        std::cout << "[remeshing] helix free!\n";
    }
}

}
