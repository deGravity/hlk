#include <directional/dual_cycles.h>
#include <directional/index_prescription.h>
#include <directional/read_raw_field.h>
#include <directional/representative_to_raw.h>
#include <directional/rotation_to_representative.h>
#include <directional/polyvector_field.h>
#include <directional/polyvector_to_raw.h>
#include <directional/write_raw_field.h>
#include <igl/AABB.h>
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/copyleft/comiso/nrosy.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/local_basis.h>
#include <igl/principal_curvature.h>
#include <igl/rotate_vectors.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/triangle_triangle_adjacency.h>

#include "meshing_algorithms.h"
#include "remeshing_plugin.h"

namespace hlk {
bool RemeshingMenu::load_raw_field() {
    std::string fname = igl::file_dialog_open();
    if (fname.length() == 0) return false;

    if (miq_mode == MIQMode::INDEX) {
        directional::read_raw_field(fname, N, rawField);
    } else {
        directional::read_raw_field(fname, rosy, curlRawField);
        if (miq_mode == MIQMode::CROSS) {
            direction_field[1] = curlRawField.block(0, 0, F.rows(), 3);
        }
    }

    has_direction_field = true;
    has_curl = false;
    return true;
}

bool RemeshingMenu::save_raw_field() {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0) return false;

    if (miq_mode == MIQMode::INDEX) {
        return directional::write_raw_field(fname, rawField);
    } else if (miq_mode == MIQMode::CROSS) {
        directional::representative_to_raw(V, F, direction_field[1], rosy, curlRawField);
    }
    return directional::write_raw_field(fname, curlRawField);
}

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
        if (face_vectors[i].assigned[direction]) continue;
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
    if (miq_mode == MIQMode::POLYVECTOR) {
        // Interpolate both directions separately first. <- WHY DO I NEED THIS????
        /*interpolate_cross_field(S, 0);
        update_vectors_from_field(0);
        interpolate_cross_field(S, 1);
        update_vectors_from_field(1);*/

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
        directional::polyvector_to_raw(V, F, polyvector_field, rosy, curlRawField);

    } else if (miq_mode == MIQMode::CROSS) {

        Eigen::VectorXd S;
        interpolate_cross_field(S);
        update_vectors_from_field();
        directional::representative_to_raw(V, F, direction_field[1], rosy, curlRawField);

        int s_count = 0;
        for (int i = 0; i < S.rows(); ++i) {
            s_count += abs(S(i)) > 0.001 ? 1 : 0;
        }
        std::cout << "Singularity Count = " << s_count << "\n";
    }

    has_direction_field = true;
    has_integer_grid = false;
    has_curl = false;
    viewing_mode = ViewingMode::MESH_ONLY;
    update_visualization();
}

void RemeshingMenu::generate_integer_grid() {

    if (miq_mode == MIQMode::INDEX) {
        if (N == 1) {
            directional::representative_to_raw(V, F, rawField, rosy, rawField);
        } else if (N != 4) {
            std::cout << "unsupported N=" << N << "\n";
            return;
        }
        Meshing::polyvector_parametrize(
            V, F, rosy, EV, EF, FE,
            rawField, combedField,
            combedMatching, combedEffort,
            singVertices, singIndices,
            VMeshCut, FMeshCut, cutUV,
            1. / gradient_size, isInteger);

    } else if (miq_mode == MIQMode::POLYVECTOR) {
        Meshing::polyvector_parametrize(
            V, F, rosy, EV, EF, FE,
            curlRawField, combedField,
            combedMatching, combedEffort,
            curlSingVertices, curlSingIndices,
            VMeshCut, FMeshCut, cutUV,
            1. / gradient_size, isInteger);

    } else { // miq_mode == MIQMode::CROSS
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
    viewing_mode = ViewingMode::MESH_ONLY;
    update_visualization();
}

void RemeshingMenu::init_curl() {
    Meshing::init_curl(
        V, F, rosy, EV, EF, FE,
        c_b, c_bc, c_blevel,
        curlRawField, combedField,
        combedMatching, combedEffort,
        curl, curlSingVertices, curlSingIndices,
        AE2F, curlMax, curlMaxOrig);

    has_curl = true;
    if (viewing_mode == ViewingMode::MESH_CURL || viewing_mode == ViewingMode::MESH_FIELD) {
        update_visualization();
    }
}

void RemeshingMenu::reduce_curl() {
    if (!has_curl) { init_curl(); }

    Meshing::reduce_curl(
        V, F, rosy, EV, EF, FE,
        curlRawField, combedField,
        combedMatching, combedEffort,
        curl, curlSingVertices, curlSingIndices,
        curlMax);

    if (miq_mode == MIQMode::CROSS) {
        direction_field[1] = combedField.block(0, 0, F.rows(), 3);
    }
    if (viewing_mode == ViewingMode::MESH_CURL || viewing_mode == ViewingMode::MESH_FIELD) {
        update_visualization();
    }
}

void RemeshingMenu::init_quad_seams() {
    if (!is_quad_meshed) { return; }

    igl::AABB<Eigen::MatrixXd, 3> aabb_tree;
    aabb_tree.init(quad_mesh.V, quad_mesh.F_t);

    quad_mesh.is_seam_edge.clear();
    quad_mesh.is_seam_edge = std::vector<bool>(4 * quad_mesh.m, false);
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
        for (int face : quad_mesh.singular_quads) { // for debugging purposes
            interactive_colors.row(face) = Eigen::RowVector3d(1., 0., 0.6);
        }
        stylize_quad_mesh(interactive_colors);

    } else {
        std::cout << "[remeshing] helix free!\n";
        stylize_quad_mesh(Eigen::RowVector3d::Constant(1.0));
    }
}

void RemeshingMenu::setup_basis_cycles() {
    directional::dual_cycles(V, F, EV, EF, basisCycles, cycleCurvature, vertex2cycle, innerEdges);
    cycleIndices = Eigen::VectorXi::Constant(basisCycles.rows(), 0);

    for (int i = 0; i < singVertices.size(); i++)
        cycleIndices(vertex2cycle(singVertices(i))) = singIndices(i);

    std::vector<std::vector<int>> boundaryLoops;
    igl::boundary_loop(F, boundaryLoops);
    numBoundaries = boundaryLoops.size();
    eulerChar = V.rows() - EV.rows() + F.rows();
    numGenerators = 2 - eulerChar - boundaryLoops.size();

    std::cout << "Euler characteristic: " << eulerChar << std::endl;
    std::cout << "#generators: " << numGenerators << std::endl;
    std::cout << "#boundaries: " << numBoundaries << std::endl;

    // collecting cycle faces for visualization
    cycleFaces.resize(basisCycles.rows());
    for (int k = 0; k < basisCycles.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(basisCycles, k); it; ++it) {
            int f1 = EF(innerEdges(it.col()), 0);
            int f2 = EF(innerEdges(it.col()), 1);
            if (f1 != -1)
                cycleFaces[it.row()].push_back(f1);
            if (f2 != -1)
                cycleFaces[it.row()].push_back(f2);
        }
    }
}

void RemeshingMenu::compute_target_curvature() {
    // the difference in the angle representation of edge i from EF(i,0) to EF(i,1)
    Eigen::VectorXd edgeParallelAngleChange(basisCycles.cols());
    for (int i = 0; i < innerEdges.rows(); i++) {
        int currEdge = innerEdges(i);
        Eigen::RowVectorXd edgeVectors = (V.row(EV(currEdge, 1)) - V.row(EV(currEdge, 0))).normalized();
        // instead of local basis, use the cross field
        double x1 = edgeVectors.dot(face_vectors[EF(currEdge, 0)].base_vector);
        double y1 = edgeVectors.dot(face_vectors[EF(currEdge, 0)].frame[1]);
        double x2 = edgeVectors.dot(face_vectors[EF(currEdge, 1)].base_vector);
        double y2 = edgeVectors.dot(face_vectors[EF(currEdge, 1)].frame[1]);
        edgeParallelAngleChange(i) = atan2(y2, x2) - atan2(y1, x1);
    }
    targetCurvature = basisCycles * edgeParallelAngleChange;
    for (int i = 0; i < targetCurvature.size(); i++) {
        while (targetCurvature(i) >= M_PI) targetCurvature(i) -= 2.0 * M_PI;
        while (targetCurvature(i) < -M_PI) targetCurvature(i) += 2.0 * M_PI;
    }
}

void RemeshingMenu::update_raw_field() {
    int sum = round(cycleIndices.head(cycleIndices.size() - numGenerators).sum());
    if (eulerChar * N != sum) {
        std::cout << "Warning: All non-generator singularities should add up to N * the Euler characteristic." << std::endl;
        std::cout << "Total indices: " << sum << "/" << N << std::endl;
        std::cout << "Expected: " << eulerChar * N << "/" << N << std::endl;
        return;
    }

    //compute_target_curvature();
    //directional::index_prescription(
    //    V, F, EV, innerEdges, basisCycles, targetCurvature,
    //    cycleCurvature, cycleIndices, ldltSolver, N, field_guidance_weight,
    //    rotationAngles, linfError);
    Eigen::VectorXd rotationAngles;
    double linfError;
    directional::index_prescription(
        V, F, EV, innerEdges, basisCycles,
        cycleCurvature, cycleIndices, ldltSolver, N,
        rotationAngles, linfError);
    std::cout << "Index prescription linfError: " << linfError << std::endl;

    directional::rotation_to_representative(V, F, EV, EF, rotationAngles, N, globalRotation, direction_field[1]);
    directional::representative_to_raw(V, F, direction_field[1], N, rawField);
}

void RemeshingMenu::update_singularities() {
    std::vector<int> singVerticesList, singIndicesList;
    for (int i = 0; i < V.rows(); i++) {
        if (cycleIndices(vertex2cycle(i))) {
            singVerticesList.push_back(i);
            singIndicesList.push_back(cycleIndices(vertex2cycle(i)));
        }
    }
    singVertices.resize(singVerticesList.size());
    singIndices.resize(singIndicesList.size());
    for (int i = 0; i < singVerticesList.size(); i++) {
        singVertices(i) = singVerticesList[i];
        singIndices(i) = singIndicesList[i];
    }
}

}
