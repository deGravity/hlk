#include <directional/glyph_lines_raw.h>
#include <directional/seam_lines.h>
#include <directional/singularity_spheres.h>
#include <directional/visualization_schemes.h>
#include <igl/AABB.h>
#include <igl/adjacency_list.h>
#include <igl/avg_edge_length.h>
#include <igl/barycenter.h>
#include <igl/copyleft/cgal/mesh_to_polyhedron.h>
#include <igl/cut_mesh.h>
#include <igl/doublearea.h>
#include <igl/edges.h>
#include <igl/edge_topology.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/jet.h>
#include <igl/local_basis.h>
#include <igl/loop.h>
#include <igl/opengl/glfw/imgui/ImGuiTraits.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/PI.h>
#include <igl/project.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_unreferenced.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/unproject_ray.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/writeOBJ.h>

#include "extract_quad_mesh.h"
#include "meshing_algorithms.h"
#include "remeshing_plugin.h"

namespace hlk {


void RemeshingMenu::init(igl::opengl::glfw::Viewer* _viewer) {
    igl::opengl::glfw::imgui::ImGuiMenu::init(_viewer);

    _viewer->core().set_rotation_type(igl::opengl::ViewerCore::RotationType::ROTATION_TYPE_TRACKBALL);
    _viewer->core().background_color = Eigen::Vector4f(0.792f, 0.792f, 0.878f, 1.000f);
    _viewer->data().line_width = 0.1f;
    _viewer->data().point_size = 0.5f;
    _viewer->core().camera_zoom = 1.f;

    // Set up multiple meshes for visualization.
    _viewer->append_mesh(); // raw field mesh   = 1
    _viewer->append_mesh(); // singularity mesh = 2
    _viewer->append_mesh(); // seam mesh        = 3
    _viewer->append_mesh(); // quad mesh        = 4
    _viewer->selected_data_index = 0;

	// This function is called every time a keyboard button is pressed
	auto key_down = [&](igl::opengl::glfw::Viewer & viewer, unsigned char key, int modifier) {
        if ((unsigned int)key == 85) czsl.ctrl = true;
        else if ((unsigned int)key == 90) czsl.z = true;
        else if ((unsigned int)key == 83) czsl.s = true;
        else if ((unsigned int)key == 76) czsl.l = true;
        else if (key == '0') singularitySelect = true;
        else if (key == 'W') {
            if (save_raw_field()) {
                std::cout << "Saved raw field.\n";
            } else {
                std::cout << "Unable to save raw field. Error: " << errno << "\n";
            }
        } else {
            bool should_draw = false;
            if (key == '1') {
                globalRotation += 0.314;
                update_raw_field();
                should_draw = true;
            } else if (key == '-' || key == '_') {
                std::vector<int> symmetric_verts = symmetrizer.symmetric_vertices(currVertex, v_symmetry_axes());
                for (int v : symmetric_verts) { cycleIndices(vertex2cycle(v))--; }
                std::cout << std::endl;
                update_raw_field();
                update_singularities();
                should_draw = true;
            } else if (key == '+' || key == '=') {
                std::vector<int> symmetric_verts = symmetrizer.symmetric_vertices(currVertex, v_symmetry_axes());
                for (int v : symmetric_verts) { cycleIndices(vertex2cycle(v))++; }
                update_raw_field();
                update_singularities();
                should_draw = true;
            } else if (key == 'B') {
                if (numBoundaries) {
                    // loop through the boundary cycles.
                    if (currCycle >= basisCycles.rows() - numBoundaries - numGenerators && currCycle < basisCycles.rows() - numGenerators - 1) {
                        currCycle++;
                    } else {
                        currCycle = basisCycles.rows() - numBoundaries - numGenerators;
                    }
                    should_draw = true;
                }
            } else if (key == 'G') {
                if (numGenerators) {
                    // loop through the generators cycles.
                    if (currCycle >= basisCycles.rows() - numGenerators && currCycle < basisCycles.rows() - 1) {
                        currCycle++;
                    } else {
                        currCycle = basisCycles.rows() - numGenerators;
                    }
                    should_draw = true;
                }
            } else if (key == 'R') {
                should_draw = true;
            }
            if (should_draw) {
                viewing_mode = ViewingMode::MESH_SING;
                update_visualization(key);
            }
        }

		if (czsl.ctrl && czsl.s) viewer.open_dialog_save_mesh();
		if (czsl.ctrl && czsl.l) viewer.open_dialog_load_mesh();
		if (czsl.ctrl && czsl.z) {
			if (temps.size() > 1) {
				load(temps[temps.size()-2].mesh);
				remove(temps.back().edge.c_str());
				remove(temps.back().face.c_str());
				remove(temps.back().mesh.c_str());
				temps.erase(temps.begin()+temps.size()-1);
			}
		}

		return false;
	};

	auto key_up = [&](igl::opengl::glfw::Viewer & viewer, unsigned char key, int modifier) {
        if ((unsigned int)key == 85) czsl.ctrl = false;
        else if ((unsigned int)key == 90) czsl.z = false;
        else if ((unsigned int)key == 83) czsl.s = false;
        else if ((unsigned int)key == 76) czsl.l = false;
        else if (key == '0') singularitySelect = false;
		return false;
	};

	_viewer->callback_key_down = key_down;
	_viewer->callback_key_up = key_up;

    if (input_model.empty()) {
#ifdef HAISEN
	    load("E:\\Stitchgraph\\stitchgraph\\data\\clothing_models\\sweater.obj");
	    //load("E:\\Stitchgraph\\stitchgraph\\data\\clothing_models\\torus.obj");
#endif
#ifdef YUXUAN
        load("/Users/Bluefish_/Desktop/01_SU19/stitchgraph/data/clothing_models/sweater.obj");
#endif
#ifdef WINDOWS_YUXUAN
        load("C:\\Users\\ym2552\\Desktop\\stitchgraph\\data\\clothing_models\\jumper.obj");
#endif
    } else {
        load(input_model);
    }
}

void RemeshingMenu::draw_viewer_menu() {
    float w = ImGui::GetContentRegionAvailWidth();
    float p = ImGui::GetStyle().FramePadding.x;
    if (ImGui::CollapsingHeader("Workspace", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Load##Workspace", ImVec2((w - p) / 2.f, 0))) {
            load_workspace();
        }
        ImGui::SameLine(0, p);
        if (ImGui::Button("Save##Workspace", ImVec2((w - p) / 2.f, 0))) {
            save_workspace();
        }
    }
    if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Load##Mesh", ImVec2((w - p) / 2.f, 0))) {
            viewer->open_dialog_load_mesh();
        }
        ImGui::SameLine(0, p);
        if (ImGui::Button("Save##Mesh", ImVec2((w - p) / 2.f, 0))) {
            viewer->open_dialog_save_mesh();
        }
        if (ImGui::Button("Load Field##Mesh", ImVec2((w - p) / 2.f, 0))) {
            load_raw_field();
        }
        ImGui::SameLine(0, p);
        if (ImGui::Button("Save Field##Mesh", ImVec2((w - p) / 2.f, 0))) {
            save_raw_field();
        }
        if (ImGui::Button("Clear Loops##Mesh", ImVec2((w - p) / 2.f, 0))) {
            clear_loops();
            update_visualization();
        }
        ImGui::SameLine(0, p);
        if (ImGui::Button("Reset Field##Mesh", ImVec2((w - p) / 2.f, 0))) {
            reset_field();
            update_visualization();
        }
        if (ImGui::Button("Subdivide Mesh", ImVec2(w - p, 0))) {
            apply_subdivision();
        }
    }

    if (ImGui::CollapsingHeader("Remeshing", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (model_loaded()) {
            // Expose symmetry variables.
            ImGui::PushItemWidth(80 * menu_scaling());
            if (symmetrizer.has_vertex_symmetry(2)) {
                ImGui::Checkbox("XY Symmetry", &symmetry_mode_xy);
            } else {
                symmetry_mode_xy = false;
            }
            if (symmetrizer.has_vertex_symmetry(1)) {
                ImGui::Checkbox("XZ Symmetry", &symmetry_mode_xz);
            } else {
                symmetry_mode_xz = false;
            }
            if (symmetrizer.has_vertex_symmetry(0)) {
                ImGui::Checkbox("YZ Symmetry", &symmetry_mode_yz);
            } else {
                symmetry_mode_yz = false;
            }
            ImGui::Checkbox("Symmetrize N-RoSy", &symmetrize_nrosy);
            ImGui::Checkbox("Show Axis", &show_axis);
            ImGui::Checkbox("Multi Points", &multi_points_drawing);
            ImGui::Checkbox("Do matching", &do_matching);
            ImGui::Checkbox("Use Guiding Field", &use_guiding_field);
            // Add threshold values.
            ImGui::DragFloat("Click Threshold", &click_threshold, 0.05f, 0.01f, 0.5f);
            ImGui::DragFloat("Soft Weight", &soft_constraint_strength, 0.0f, 0.0f, 1.0f);
            if (use_guiding_field) {
                ImGui::DragFloat("Field Weight", &field_guidance_weight, 0.0f, 0.0f, 1.0f);
            }
            ImGui::DragFloat("Gradient Size", &gradient_size, 1.0f, 0.0f, 150.0f);
            ImGui::DragInt("# Stiffening", &stiffen_iter, 1, 0, 10);
            if (miq_mode == MIQMode::INDEX) {
                ImGui::DragInt("N-vector field", &N, 1, 1, 4);
            }
            ImGui::PopItemWidth();
            ImGui::Text("===Field Operations===");
            /*if (ImGui::Button("Initialize field from principal curvatures", ImVec2(w - p, 0))) {
                init_curvature_field();
                viewing_mode = ViewingMode::MESH_ONLY; update_visualization();
            }*/
            // Direction field controls
            if (has_direction_field) {
                // field interpolation mode options.
                ImGui::Text("Click to select the type of field.");
                if (ImGui::RadioButton("Cross", miq_mode == MIQMode::CROSS)) {
                    miq_mode = MIQMode::CROSS;
                }
                ImGui::SameLine(0, p);
                if (ImGui::RadioButton("Polyvec", miq_mode == MIQMode::POLYVECTOR)) {
                    miq_mode = MIQMode::POLYVECTOR;
                }
                ImGui::SameLine(0, p);
                if (ImGui::RadioButton("Index", miq_mode == MIQMode::INDEX)) {
                    miq_mode = MIQMode::INDEX;
                }
                if (miq_mode != MIQMode::INDEX) {
                    if (ImGui::Button("Interpolate Field", ImVec2(w - p, 0))) {
                        interpolate_field();
                    }
                    if (ImGui::Button("Init Curl", ImVec2((w - p) / 2.f, 0))) {
                        init_curl();
                    }
                    ImGui::SameLine(0, p);
                    if (ImGui::Button("Reduce Curl", ImVec2((w - p) / 2.f, 0))) {
                        reduce_curl();
                    }
                }
                if (ImGui::Button("Run MIQ parametrization", ImVec2(w - p, 0))) {
                    generate_integer_grid();
                }
            }
            // Quad controls
            ImGui::Text("===Quad Operations===");
            if (has_integer_grid) {
                if (ImGui::Button("Extract Quad Mesh", ImVec2(w - p, 0))) {
                    std::vector<std::vector<double>> Vs, TCs;
                    std::vector<std::vector<int>> Fs;
                    if (miq_mode == MIQMode::CROSS) {
                        extract_quad_mesh(V, F, V_uv, F_uv, quad_mesh);
                    } else {
                        extract_quad_mesh(V, F, cutUV, FMeshCut, quad_mesh);
                    }
                    is_quad_meshed = true;
                    init_quad_seams();
                    viewing_mode = ViewingMode::QUAD_ONLY;
                    update_visualization();
                }
            }
            if (is_quad_meshed) {
                if (ImGui::Button("Save Quads", ImVec2((w - p) / 2.f, 0))) {
                    if (is_quad_meshed) {
                        std::string fname = igl::file_dialog_save();
                        if (fname.length() > 0) {
                            Eigen::MatrixXd V_q = quad_mesh.V.block(0, 0, quad_mesh.n, 3);
                            igl::writeOBJ(fname, V_q, quad_mesh.F_q);
                        }
                    }
                }
                ImGui::SameLine(0, p);
                if (ImGui::Button("Check Helix", ImVec2((w - p) / 2.f, 0))) {
                    quad_helix_finding();
                }
            }
        }
    }

    // Visualization Options
    if (ImGui::CollapsingHeader("Visualization Options", ImGuiTreeNodeFlags_DefaultOpen)) {
        // drawing mode options.
        ImGui::Text("Click to select the drawing mode.");
        if (ImGui::RadioButton("Course", drawing_mode == DrawingMode::COURSE)) {
            drawing_mode = DrawingMode::COURSE;
        }
        ImGui::SameLine(0, p);
        if (ImGui::RadioButton("Wale", drawing_mode == DrawingMode::WALE)) {
            drawing_mode = DrawingMode::WALE;
        }
        // viewing mode options.
        ImGui::Text("Click to select the viewing mode.");
        if (ImGui::RadioButton("Mesh Only", viewing_mode == ViewingMode::MESH_ONLY)) {
            viewing_mode = ViewingMode::MESH_ONLY; update_visualization();
        }
        ImGui::SameLine(0, p);
        if (ImGui::RadioButton("Mesh+Sing", viewing_mode == ViewingMode::MESH_SING)) {
            viewing_mode = ViewingMode::MESH_SING; update_raw_field(); update_visualization();
        }
        if (ImGui::RadioButton("Mesh+Field", viewing_mode == ViewingMode::MESH_FIELD)) {
            viewing_mode = ViewingMode::MESH_FIELD; update_visualization();
        }
        ImGui::SameLine(0, p);
        if (ImGui::RadioButton("Mesh+Curl", viewing_mode == ViewingMode::MESH_CURL)) {
            viewing_mode = ViewingMode::MESH_CURL; update_visualization();
        }
        if (is_quad_meshed) {
            if (ImGui::RadioButton("Mesh+Quad", viewing_mode == ViewingMode::MESH_QUAD)) {
                viewing_mode = ViewingMode::MESH_QUAD; update_visualization();
            }
            ImGui::SameLine(0, p);
            if (ImGui::RadioButton("Quad Only", viewing_mode == ViewingMode::QUAD_ONLY)) {
                viewing_mode = ViewingMode::QUAD_ONLY; update_visualization();
            }
        }
    }

    // Viewing options
    if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_OpenOnArrow)) {
        if (ImGui::Button("Center object", ImVec2(-1, 0))) {
            viewer->core().align_camera_center(V, F);
        }
        if (ImGui::Button("Snap canonical view", ImVec2(-1, 0))) {
            viewer->snap_to_canonical_quaternion();
        }
        // Zoom
        ImGui::PushItemWidth(80 * menu_scaling());
        ImGui::DragFloat("Zoom", &(viewer->core().camera_zoom), 0.05f, 0.1f, 20.0f);
        // Select rotation type
        int rotation_type = static_cast<int>(viewer->core().rotation_type);
        static Eigen::Quaternionf trackball_angle = Eigen::Quaternionf::Identity();
        static bool orthographic = true;
        if (ImGui::Combo("Camera Type", &rotation_type, "Trackball\0Two Axes\0002D Mode\0\0")) {
            using RT = igl::opengl::ViewerCore::RotationType;
            auto new_type = static_cast<RT>(rotation_type);
            if (new_type != viewer->core().rotation_type) {
                if (new_type == RT::ROTATION_TYPE_NO_ROTATION) {
                    trackball_angle = viewer->core().trackball_angle;
                    orthographic = viewer->core().orthographic;
                    viewer->core().trackball_angle = Eigen::Quaternionf::Identity();
                    viewer->core().orthographic = true;
                } else if (viewer->core().rotation_type == RT::ROTATION_TYPE_NO_ROTATION) {
                    viewer->core().trackball_angle = trackball_angle;
                    viewer->core().orthographic = orthographic;
                }
                viewer->core().set_rotation_type(new_type);
            }
        }
        // Orthographic view
        ImGui::Checkbox("Orthographic view", &(viewer->core().orthographic));
        ImGui::PopItemWidth();
    }

    // Helper for setting viewport specific mesh options
    auto make_checkbox = [&](const char* label, unsigned int& option) {
        return ImGui::Checkbox(label,
            [&]() { return viewer->core().is_set(option); },
            [&](bool value) { return viewer->core().set(option, value); });
    };
    // Draw options
    if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_OpenOnArrow)) {
        if (ImGui::Checkbox("Face-based", &(viewer->data().face_based))) {
            viewer->data().dirty = igl::opengl::MeshGL::DIRTY_ALL;
        }
        ImGui::Checkbox("Show Stitches", &show_stitches);
        make_checkbox("Show texture", viewer->data().show_texture);
        if (ImGui::Checkbox("Invert normals", &(viewer->data().invert_normals))) {
            viewer->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
        }
        ImGui::ColorEdit4("Background", viewer->core().background_color.data(),
            ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
        ImGui::ColorEdit4("Line color", viewer->data().line_color.data(),
            ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
        ImGui::DragFloat("Shininess", &(viewer->data().shininess), 0.05f, 0.0f, 100.0f);
        ImGui::PopItemWidth();
    }
    // Overlays
    if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_OpenOnArrow)) {
        make_checkbox("Wireframe", viewer->data().show_lines);
        make_checkbox("Fill", viewer->data().show_faces);
        make_checkbox("Show overlay", viewer->data().show_overlay);
        make_checkbox("Show overlay depth", viewer->data().show_overlay_depth);
        ImGui::Checkbox("Show vertex labels", &(viewer->data().show_vertid));
        ImGui::Checkbox("Show faces labels", &(viewer->data().show_faceid));
    }
}

void RemeshingMenu::setup_mesh() {
    // Compute face barycenters
    igl::barycenter(V, F, B);

    // Compute local basis
    igl::local_basis(V, F, B1, B2, B3);

    // Compute edge topology
    igl::edge_topology(V, F, EV, FE, EF);

    // Compute scale for visualizing fields
    global_scale = .5 * igl::avg_edge_length(V, F);

    // Pass through the symmetrizer to find symmetries.
    symmetrizer = Symmetrizer(V, F);

    viewer->data().clear();
    viewer->data().set_mesh(V, F);
    get_mesh_information();
    draw_a_point(mesh_center, 1);
}

bool RemeshingMenu::load(std::string filename) {
    // Load the mesh.
    clear();
    Eigen::MatrixXd tempV;
    Eigen::MatrixXi tempF;
    igl::read_triangle_mesh(filename, tempV, tempF);
    Eigen::VectorXi tmp1, tmp2; // Unused, needed for static igl on clang
    igl::remove_unreferenced(tempV, tempF, V, F, tmp1, tmp2);
    if (V.rows() <= 3 || F.rows() <= 1) {
        std::cerr << "Fail to load file " << filename << "...\n";
        return false;
    }
	setup_mesh();
    update_polyhedron_tree();
    reset_face_vectors();
    load_temp_data(filename);
 
    // Set up the field.
    setup_boundary();
    interpolate_field();
    setup_basis_cycles();
    
    // Other viewing settings
    viewer->core().align_camera_center(V, F);
    viewer->core().lighting_factor = 0.0;
    viewer->data().lines = Eigen::MatrixXd(0, 9);
    viewer->data().set_face_based(true);

    update_visualization();

	if (temps.empty()) save_ctrlz();

    return true;
}

void RemeshingMenu::load_temp_data(std::string filename) {
    ////////////////////////////////////////////////////////
    std::string face_path_temp;
    std::string edge_path_temp;
    std::size_t found = filename.find(".obj");
    if (found != std::string::npos) {
        face_path_temp = filename.substr(0, found) + "_temp.face";
        edge_path_temp = filename.substr(0, found) + "_temp.edge";
    } else {
        face_path_temp = filename + "_temp.face";
        edge_path_temp = filename + "_temp.edge";
        filename = filename + ".obj";
    }
    ////////////////////////////////////////////////////////

    std::ifstream ifs(face_path_temp);
    if (!ifs) {
        std::cout << "No temp data found; ignoring...\n";
        return;
    }

    int nb;
    ifs >> nb;
    for (int i = 0; i < nb; i++) {
        int face_id;
		ifs >> face_id;
		auto& face = face_vectors[face_id];
        bool assigned0, assigned1;
        ifs >> face.is_hard >> assigned0 >> assigned1
            >> face.frame[0][0] >> face.frame[0][1] >> face.frame[0][2]
			>> face.frame[1][0] >> face.frame[1][1] >> face.frame[1][2]
			>> face.base_vector[0] >> face.base_vector[1] >> face.base_vector[2]
			>> face.center[0] >> face.center[1] >> face.center[2]
            >> face.normal[0] >> face.normal[1] >> face.normal[2];
        face.assigned[0] = assigned0;
        face.assigned[1] = assigned1;
    }
    ifs.clear();
    ifs.close();

    split_edges.clear();
    ifs.open(edge_path_temp);
    ifs >> nb;
    for (int i = 0; i < nb; i++) {
        SplitEdge se;
        ifs >> se.index_0 >> se.index_1 
            >> se.normal[0] >> se.normal[1] >> se.normal[2];
        split_edges.push_back(se);
    }
    ifs.clear();
    ifs.close();
}

bool RemeshingMenu::save(std::string filename) {
    ////////////////////////////////////////////////////////
	std::string face_path_temp;
	std::string edge_path_temp;
	std::size_t found = filename.find(".obj");
	if (found != std::string::npos) {
		face_path_temp = filename.substr(0, found) + "_temp.face";
		edge_path_temp = filename.substr(0, found) + "_temp.edge";
	} else {
		face_path_temp = filename + "_temp.face";
		edge_path_temp = filename + "_temp.edge";
		filename = filename + ".obj";
	}
	////////////////////////////////////////////////////////
	// output obj
	igl::writeOBJ(filename, V, F);
	// output face_path_temp
	int face_assign_nb = 0;
	for (int i = 0; i < face_vectors.size(); i++)
		if (face_vectors[i].assigned[0] || face_vectors[i].assigned[1]) face_assign_nb++;
	std::ofstream face_file_temp(face_path_temp);
	face_file_temp << face_assign_nb << "\n";
	for (int i = 0; i < face_vectors.size(); i++) {
		if (face_vectors[i].assigned[0] || face_vectors[i].assigned[1]) {
			face_file_temp << face_vectors[i].face_id << " " 
                << face_vectors[i].is_hard << " " 
                << face_vectors[i].assigned[0] <<" "<< face_vectors[i].assigned[1] << " "
                << face_vectors[i].frame[0][0] << " " << face_vectors[i].frame[0][1] << " " << face_vectors[i].frame[0][2] << " "
                << face_vectors[i].frame[1][0] << " " << face_vectors[i].frame[1][1] << " " << face_vectors[i].frame[1][2] << " "
				<< face_vectors[i].base_vector[0] << " " << face_vectors[i].base_vector[1] << " " << face_vectors[i].base_vector[2] << " "
				<< face_vectors[i].center[0] << " " << face_vectors[i].center[1] << " " << face_vectors[i].center[2] << " "
				<< face_vectors[i].normal[0] << " " << face_vectors[i].normal[1] << " " << face_vectors[i].normal[2] << "\n";
		}
	}
	face_file_temp.clear();
	face_file_temp.close();
	// output edge_path_temp
	std::ofstream edge_file_temp(edge_path_temp);
	edge_file_temp << split_edges.size() << "\n";
	for (int i = 0; i < split_edges.size(); i++) {
		edge_file_temp << split_edges[i].index_0 << " " << split_edges[i].index_1 << " "
			<< split_edges[i].normal[0] << " " << split_edges[i].normal[1] << " " << split_edges[i].normal[2] << "\n";
	}
	edge_file_temp.clear();
	edge_file_temp.close();
	return true;
}

void RemeshingMenu::clear() {
	// feature points
	std::vector<Eigen::Vector3d>().swap(feature_points);
    std::vector<int>().swap(feature_face_ids);
    std::vector<int>().swap(loop_feature_face_ids);
	// directional faces
	std::vector<FaceVector>().swap(face_vectors);
	std::vector<SplitEdge>().swap(split_edges);
	// other geometry data
	std::vector<int>().swap(geodesic_path);
	igl_polyhedron.clear();
	igl_tree.clear();
	std::vector<std::unordered_set<int>>().swap(igl_v_faces);
	std::vector<std::vector<double>>().swap(graph_adj);
    cycleFaces.clear();
    clear_loops();
    // reset values
    show_axis = false;
    show_stitches = false;
    symmetry_mode_yz = false;
    symmetry_mode_xz = false;
    symmetry_mode_xy = false;
    symmetrize_nrosy = false;
    has_direction_field = false;
    has_integer_grid = false;
    has_curl = false;
    is_quad_meshed = false;
    should_redraw = false;
    do_matching = false;
    use_guiding_field = false;
    viewing_mode = ViewingMode::MESH_ONLY;
    drawing_mode = DrawingMode::WALE;
    miq_mode = MIQMode::CROSS;
    cardinal = Cardinal::N;
    direction_field = { Eigen::MatrixXd(), Eigen::MatrixXd() };
}

/////////////////////////// STATE SERIALIZATION ////////////////////////////

// equivalent to serialize().
bool RemeshingMenu::save_workspace() {
    std::string filename = igl::file_dialog_save();
    if (filename.length() == 0) return false;

    igl::serialize(has_direction_field, "has_direction_field", filename);
    igl::serialize(has_curl, "has_curl", filename);
    igl::serialize(has_integer_grid, "has_integer_grid", filename);
    igl::serialize(is_quad_meshed, "is_quad_meshed", filename);
    igl::serialize(should_redraw, "should_redraw", filename);
    igl::serialize(show_axis, "show_axis", filename);
    igl::serialize(multi_points_drawing, "multi_points_drawing", filename);
    igl::serialize(do_matching, "do_matching", filename);
    igl::serialize(use_guiding_field, "use_guiding_field", filename);
    igl::serialize(existing_edge_label, "existing_edge_label", filename);
    igl::serialize(geodesic_label, "geodesic_label", filename);

    igl::serialize(rosy, "rosy", filename);
    igl::serialize(stiffen_iter, "stiffen_iter", filename);
    igl::serialize(geodesic_index, "geodesic_index", filename);
    igl::serialize(loop_start_index, "loop_start_index", filename);
    igl::serialize(loop_end_index, "loop_end_index", filename);

    igl::serialize(cardinal, "cardinal", filename);
    igl::serialize(miq_mode, "miq_mode", filename);
    igl::serialize(drawing_mode, "drawing_mode", filename);
    igl::serialize(viewing_mode, "viewing_mode", filename);

    igl::serialize(click_threshold, "click_threshold", filename);
    igl::serialize(gradient_size, "gradient_size", filename);
    igl::serialize(soft_constraint_strength, "soft_constraint_strength", filename);
    igl::serialize(loop_size, "loop_size", filename);

    igl::serialize(in_path, "in_path", filename);
    igl::serialize(out_path, "out_path", filename);

    igl::serialize(V, "V", filename);
    igl::serialize(F, "F", filename);
    igl::serialize(V_uv, "V_uv", filename);
    igl::serialize(F_uv, "F_uv", filename);
    igl::serialize(VMeshCut, "VMeshCut", filename);
    igl::serialize(FMeshCut, "FMeshCut", filename);
    igl::serialize(cutUV, "cutUV", filename);

    igl::serialize(direction_field, "direction_field", filename);
    igl::serialize(curlRawField, "curlRawField", filename);
    igl::serialize(c_b, "c_b", filename);
    igl::serialize(c_blevel, "c_blevel", filename);
    igl::serialize(c_bc, "c_bc", filename);
    igl::serialize(polyvector_field, "polyvector_field", filename);
    igl::serialize(p_b, "p_b", filename);
    igl::serialize(p_bc, "p_bc", filename);

    if (is_quad_meshed) {
        igl::serialize(quad_mesh, "quad_mesh", filename);
    }

    igl::serialize(feature_points, "feature_points", filename);
    igl::serialize(feature_face_ids, "feature_face_ids", filename);
    igl::serialize(geodesic_point, "geodesic_point", filename);
    igl::serialize(geodesic_path, "geodesic_path", filename);
    igl::serialize(face_vectors, "face_vectors", filename);
    igl::serialize(split_edges, "split_edges", filename);

    igl::serialize(loop_points, "loop_points", filename);
    igl::serialize(loop_de_points, "loop_de_points", filename);
    igl::serialize(loop_polylines, "loop_polylines", filename);
    igl::serialize(loop_update_polylines, "loop_update_polylines", filename);
    igl::serialize(loop_path, "loop_path", filename);
    igl::serialize(loop_feature_face_ids, "loop_feature_face_ids", filename);

    igl::serialize(singVertices, "singVertices", filename);
    igl::serialize(singIndices, "singIndices", filename);
    igl::serialize(rawField, "rawField", filename);
    igl::serialize(cycleIndices, "cycleIndices", filename);
    igl::serialize(cycleCurvature, "cycleCurvature", filename);
    igl::serialize(targetCurvature, "targetCurvature", filename);
    igl::serialize(basisCycles, "basisCycles", filename);
    igl::serialize(vertex2cycle, "vertex2cycle", filename);
    igl::serialize(innerEdges, "innerEdges", filename);
    igl::serialize(cycleFaces, "cycleFaces", filename);
    igl::serialize(eulerChar, "eulerChar", filename);
    igl::serialize(numGenerators, "numGenerators", filename);
    igl::serialize(numBoundaries, "numBoundaries", filename);
    igl::serialize(currVertex, "currVertex", filename);
    igl::serialize(currCycle, "currCycle", filename);
    igl::serialize(N, "N", filename);
    igl::serialize(globalRotation, "globalRotation", filename);

    return true;
}

// equivalent to deserialize().
bool RemeshingMenu::load_workspace() {
    std::string filename = igl::file_dialog_open();
    if (filename.length() == 0) return false;

    clear();

    igl::deserialize(has_direction_field, "has_direction_field", filename);
    igl::deserialize(has_curl, "has_curl", filename);
    igl::deserialize(has_integer_grid, "has_integer_grid", filename);
    igl::deserialize(is_quad_meshed, "is_quad_meshed", filename);
    igl::deserialize(should_redraw, "should_redraw", filename);
    igl::deserialize(show_axis, "show_axis", filename);
    igl::deserialize(multi_points_drawing, "multi_points_drawing", filename);
    igl::deserialize(do_matching, "do_matching", filename);
    igl::deserialize(use_guiding_field, "use_guiding_field", filename);
    igl::deserialize(existing_edge_label, "existing_edge_label", filename);
    igl::deserialize(geodesic_label, "geodesic_label", filename);

    igl::deserialize(rosy, "rosy", filename);
    igl::deserialize(stiffen_iter, "stiffen_iter", filename);
    igl::deserialize(geodesic_index, "geodesic_index", filename);
    igl::deserialize(loop_start_index, "loop_start_index", filename);
    igl::deserialize(loop_end_index, "loop_end_index", filename);

    igl::deserialize(cardinal, "cardinal", filename);
    igl::deserialize(miq_mode, "miq_mode", filename);
    igl::deserialize(drawing_mode, "drawing_mode", filename);
    igl::deserialize(viewing_mode, "viewing_mode", filename);

    igl::deserialize(click_threshold, "click_threshold", filename);
    igl::deserialize(gradient_size, "gradient_size", filename);
    igl::deserialize(soft_constraint_strength, "soft_constraint_strength", filename);
    igl::deserialize(loop_size, "loop_size", filename);

    igl::deserialize(in_path, "in_path", filename);
    igl::deserialize(out_path, "out_path", filename);

    igl::deserialize(V, "V", filename);
    igl::deserialize(F, "F", filename);
    igl::deserialize(V_uv, "V_uv", filename);
    igl::deserialize(F_uv, "F_uv", filename);
    igl::deserialize(VMeshCut, "VMeshCut", filename);
    igl::deserialize(FMeshCut, "FMeshCut", filename);
    igl::deserialize(cutUV, "cutUV", filename);

    setup_mesh();
    update_polyhedron_tree();

    igl::deserialize(direction_field, "direction_field", filename);
    igl::deserialize(curlRawField, "curlRawField", filename);
    igl::deserialize(c_b, "c_b", filename);
    igl::deserialize(c_blevel, "c_blevel", filename);
    igl::deserialize(c_bc, "c_bc", filename);
    igl::deserialize(polyvector_field, "polyvector_field", filename);
    igl::deserialize(p_b, "p_b", filename);
    igl::deserialize(p_bc, "p_bc", filename);

    if (is_quad_meshed) {
        igl::deserialize(quad_mesh, "quad_mesh", filename);
    }

    igl::deserialize(feature_points, "feature_points", filename);
    igl::deserialize(feature_face_ids, "feature_face_ids", filename);
    igl::deserialize(geodesic_point, "geodesic_point", filename);
    igl::deserialize(geodesic_path, "geodesic_path", filename);
    igl::deserialize(face_vectors, "face_vectors", filename);
    igl::deserialize(split_edges, "split_edges", filename);

    igl::deserialize(loop_points, "loop_points", filename);
    igl::deserialize(loop_de_points, "loop_de_points", filename);
    igl::deserialize(loop_polylines, "loop_polylines", filename);
    igl::deserialize(loop_update_polylines, "loop_update_polylines", filename);
    igl::deserialize(loop_path, "loop_path", filename);
    igl::deserialize(loop_feature_face_ids, "loop_feature_face_ids", filename);

    igl::deserialize(singVertices, "singVertices", filename);
    igl::deserialize(singIndices, "singIndices", filename);
    igl::deserialize(rawField, "rawField", filename);
    igl::deserialize(cycleIndices, "cycleIndices", filename);
    igl::deserialize(cycleCurvature, "cycleCurvature", filename);
    igl::deserialize(targetCurvature, "targetCurvature", filename);
    igl::deserialize(basisCycles, "basisCycles", filename);
    igl::deserialize(vertex2cycle, "vertex2cycle", filename);
    igl::deserialize(innerEdges, "innerEdges", filename);
    igl::deserialize(cycleFaces, "cycleFaces", filename);
    igl::deserialize(eulerChar, "eulerChar", filename);
    igl::deserialize(numGenerators, "numGenerators", filename);
    igl::deserialize(numBoundaries, "numBoundaries", filename);
    igl::deserialize(currVertex, "currVertex", filename);
    igl::deserialize(currCycle, "currCycle", filename);
    igl::deserialize(N, "N", filename);
    igl::deserialize(globalRotation, "globalRotation", filename);

    line_texture(texture_R, texture_G, texture_B);

    update_visualization();

    if (temps.empty()) save_ctrlz();

    return true;
}

////////////////////////////// UI MAIN LOGIC ///////////////////////////////

bool RemeshingMenu::mouse_down(int button, int modifier) {
    if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_down(button, modifier)) return true;

	mouse_key = button;
	ctrl_on = modifier & IGL_MOD_CONTROL;
	alt_on = modifier & IGL_MOD_ALT;
	shift_on = modifier & IGL_MOD_SHIFT;
	mouse_down_on = true;
	mouse_x = viewer->current_mouse_x;
	mouse_y = viewer->core().viewport(3) - viewer->current_mouse_y;

	return false;
}

bool RemeshingMenu::mouse_move(int mouse_x, int mouse_y) {
    if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_move(mouse_x, mouse_y)) return true;

	if (model_loaded()) {
		if ((mouse_key == 0|| mouse_key == 1|| mouse_key == 2) && mouse_down_on) { // left/right click-and-drag
            double x = viewer->current_mouse_x;
            double y = viewer->core().viewport(3) - viewer->current_mouse_y;
            int fid;
            Eigen::Vector3f bc;
            if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer->core().view,
                viewer->core().proj, viewer->core().viewport, V, F, fid, bc)) {
                if ((ctrl_on && !alt_on && !shift_on && mouse_key == 0) || // ctrl+left, red
                    (!ctrl_on && alt_on && !shift_on && mouse_key == 0) || // alt+left, green
                    (!ctrl_on && !alt_on && shift_on && mouse_key == 0) || // shift+left, blue/orange
                    (!ctrl_on && alt_on && !shift_on && mouse_key == 1) || // alt+middle, green
                    ( ctrl_on && alt_on && !shift_on && mouse_key == 2) || // ctrl+alt+right, black
                    (!ctrl_on && alt_on && !shift_on && mouse_key == 2)) { // alt+right, green

					// draw stroke if intersecting
					const Eigen::Vector3d intersection =
						(V.row(F(fid, 0)) * bc(0) + V.row(F(fid, 1)) * bc(1) + V.row(F(fid, 2)) * bc(2)).transpose();
					Eigen::Vector3d normal = face_vectors[fid].normal;
					feature_points.emplace_back(intersection);
					loop_feature_face_ids.emplace_back(fid);
					if (feature_points.size() >= 2) {
						Eigen::Vector3d src = feature_points[0] + normal * mesh_size * 0.001;
						Eigen::Vector3d dst = feature_points[feature_points.size() - 1] + normal * mesh_size * 0.001;
						std::vector<Eigen::Vector3d> vecs = { src, dst };
						
                        if (ctrl_on && alt_on) {
                            draw_points(vecs, 4);
                        } else if (ctrl_on) {
                            draw_points(vecs, 0);
						} else if (alt_on) {
                            if (mouse_key == 0) { draw_points(vecs, 1); }
                            if (mouse_key == 1 || mouse_key == 2) {
                                viewer->data().points = Eigen::MatrixXd(0, 6);
                                viewer->data().lines = Eigen::MatrixXd(0, 9);
                                draw_segments(vecs, 1, 0.0);
                                draw_points(vecs, 1, 0.0);
                            }
						} else if (shift_on) {
                            draw_points(vecs, drawing_mode == DrawingMode::COURSE ? 2 : 3);
                        }
						should_redraw = true;
					}
					return true;
				}
			} else { // discard stroke data
				if (!feature_points.empty() || !loop_feature_face_ids.empty()) {
					should_redraw = true;
                }
				feature_points.clear();
                loop_feature_face_ids.clear();
				if (should_redraw) {
                    update_visualization();
					should_redraw = false;
				}
				return false;
			}
		}
	}
	return false;
}

bool RemeshingMenu::mouse_scroll(float delta_y) {
    if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_scroll(delta_y)) return true;

	if (model_loaded()) {
		float x = viewer->core().viewport(2) / 2.0;
        float y = viewer->core().viewport(3) / 2.0;

		Eigen::Vector3f s, dir;
		igl::unproject_ray(Eigen::Vector2f(x, y), viewer->core().view,
			viewer->core().proj, viewer->core().viewport, s, dir);
		dir = s - mesh_center.cast<float>();
		Eigen::Vector3f n(1.0, 1.0, 1.0);
		if (!is_almost_zero(dir[0])) {
			n[0] = -(dir[1] + dir[2]) / dir[0];
		}
		else if (!is_almost_zero(dir[1])) {
			n[1] = -(dir[0] + dir[2]) / dir[1];
		}
		else if (!is_almost_zero(dir[2])) {
			n[2] = -(dir[0] + dir[1]) / dir[2];
		}
		Eigen::Vector3f base = n.normalized();

		Eigen::Vector3f point_center = mesh_center.cast<float>();
		Eigen::Vector3f point_0 = mesh_center.cast<float>() + base * (float)mesh_size * 0.002f;
		Eigen::Vector3f point_1 = mesh_center.cast<float>() - base * (float)mesh_size * 0.002f;

		Eigen::Matrix<float, 3, 1> obj_center(
			point_center.x(), point_center.y(), point_center.z());
		Eigen::Matrix<float, 3, 1> obj_0(point_0.x(), point_0.y(), point_0.z());
		Eigen::Matrix<float, 3, 1> obj_1(point_1.x(), point_1.y(), point_1.z());

		Eigen::Matrix<float, 3, 1> project_point_center =
			igl::project(obj_center, viewer->core().view, viewer->core().proj, viewer->core().viewport);
		Eigen::Matrix<float, 3, 1> project_point_0 =
			igl::project(obj_0, viewer->core().view, viewer->core().proj, viewer->core().viewport);
		Eigen::Matrix<float, 3, 1> project_point_1 =
			igl::project(obj_1, viewer->core().view, viewer->core().proj, viewer->core().viewport);

		double distance = (project_point_0 - project_point_1).norm();
		viewer->data().line_width = distance * 0.4;
		viewer->data().point_size = distance * 4.0;
	}
	return false;
}

bool RemeshingMenu::mouse_up(int button, int modifier) {
    if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_up(button, modifier)) return true;

	mouse_down_on = false;

	if (!model_loaded()) return true;
	if (button == 2 && (modifier & IGL_MOD_SHIFT)) { // shift+right
		if (highlight_symmetries()) return true;
	}

    double x = viewer->current_mouse_x;
    double y = viewer->core().viewport(3) - viewer->current_mouse_y;
    double mouse_d = std::sqrt(std::pow(x-mouse_x, 2) + std::pow(y-mouse_y, 2));
    int fid;
    Eigen::Vector3f bc;
    bool intersects = igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer->core().view,
        viewer->core().proj, viewer->core().viewport, V, F, fid, bc);

    if (button == 0 && singularitySelect) { // trivial connections: select singularity
        Eigen::Vector3d::Index maxCol;
        bc.maxCoeff(&maxCol);
        currVertex = F(fid, maxCol);
        currCycle = vertex2cycle(currVertex);
        viewing_mode = ViewingMode::MESH_SING;
        should_redraw = true;
        std::cout << "sing selected\n";

    } else if ((button == 2 || button == 1) && alt_on && !shift_on && !ctrl_on) { // loop
		if (feature_points.size() > 2) {
			auto n = face_vectors[loop_feature_face_ids[0]].normal;
            Eigen::Vector3d a = feature_points.back() - feature_points.front();

			if (button==2) {
				auto plane_n = a.cross(n).normalized();
				auto plane_p = feature_points[0];
				compute_elastic_loop_min_geodesic(plane_p, plane_n);
				//compute_elastic_loop_field_align(plane_p, plane_n);
				symmetry_elastic_loop(plane_p, plane_n);
				should_redraw = true;
			} else { // button == 1
				compute_elastic_loop(feature_points.front(),a.normalized());
			}
		}
		
	} else if (button == 2 && ctrl_on && alt_on) { // ctrl+alt+right
        if (mouse_d < 2) {
            if (intersects) {
                auto faces_to_deselect = symmetrizer.symmetric_faces(fid, f_symmetry_axes());
                for (auto face : faces_to_deselect) face_vectors[face].assigned[drawing_mode] = false;
            }
        } else if (mouse_d > 10 && feature_points.size() > 2) {
            std::vector<std::vector<int>> cutting_faces(F.rows(), std::vector<int>());
            std::vector<int> cutting_0_edges;
            std::vector<int> cutting_1_edges;
            std::vector<Eigen::Vector3d> cutting_points;
            CGAL_Mesh_Cutting(feature_points, mesh_edge_size * click_threshold,
                igl_tree, feature_face_ids, cutting_0_edges, cutting_1_edges, cutting_points, cutting_faces);

            if (!cutting_0_edges.empty()) {
                for (int fid = 0; fid < cutting_faces.size(); fid++) {
                    if (!cutting_faces[fid].empty()) {
                        auto faces_to_deselect = symmetrizer.symmetric_faces(fid, f_symmetry_axes());
                        for (auto face : faces_to_deselect) face_vectors[face].assigned[drawing_mode] = false;
                    }
                }
            }
        }

    } else if (button == 0) { // left
		if ((ctrl_on && !shift_on && !alt_on) ||
			(shift_on && !ctrl_on && !alt_on) ||
			(alt_on && !shift_on && !ctrl_on)) { // ctrl/shift/alt

			if (mouse_d < 2 && feature_points.size() < 2) {
				// geodesic_point
				if (intersects) {
					const Eigen::RowVector3d intersection =
						V.row(F(fid, 0)) * bc(0) + V.row(F(fid, 1)) * bc(1) + V.row(F(fid, 2)) * bc(2);
					double d0 = (V.row(F(fid, 0)) - intersection).norm();
					double d1 = (V.row(F(fid, 1)) - intersection).norm();
					double d2 = (V.row(F(fid, 2)) - intersection).norm();

					if (!geodesic_label) { // src point
						geodesic_point = intersection;
						geodesic_index = -1;
						if (d0 < mesh_edge_size * click_threshold) {
							geodesic_point = V.row(F(fid, 0));
							geodesic_index = F(fid, 0);
						}
						if (d1 < mesh_edge_size * click_threshold) {
							geodesic_point = V.row(F(fid, 1));
							geodesic_index = F(fid, 1);
						}
						if (d2 < mesh_edge_size * click_threshold) {
							geodesic_point = V.row(F(fid, 2));
							geodesic_index = F(fid, 2);
						}
						geodesic_label = true;
					} else { // dst point
						Eigen::Vector3d geodesic_point_1 = intersection;
						int geodesic_index_1 = -1;
						if (d0 < mesh_edge_size * click_threshold) {
							geodesic_point_1 = V.row(F(fid, 0));
							geodesic_index_1 = F(fid, 0);
						}
						if (d1 < mesh_edge_size * click_threshold) {
							geodesic_point_1 = V.row(F(fid, 1));
							geodesic_index_1 = F(fid, 1);
						}
						if (d2 < mesh_edge_size * click_threshold) {
							geodesic_point_1 = V.row(F(fid, 2));
							geodesic_index_1 = F(fid, 2);
						}
						geodesic_label = false;
						//////////////////////////////////////////////////
						feature_points.clear();
						feature_points.push_back(geodesic_point);
						feature_points.push_back(geodesic_point_1);
						/////////////////////////////////////////////
						if (geodesic_index >= 0 && geodesic_index_1 >= 0 && !existing_edge_label) {
							geodesic_path.clear();
							std::vector<int>().swap(geodesic_path);
							graph_dijkstra(graph_adj, geodesic_index, geodesic_index_1, geodesic_path);
							existing_edge_label = true;
							geodesic_index = -1;
						}
					}
				}
			}
		}

		// close all multi points selecting/seaming lines
		if (!((multi_points_drawing && feature_points.size() < 2) ||
			(!multi_points_drawing && feature_points.size() != 2))) {

			// hard constraint / soft constraint
			if ((ctrl_on && !shift_on && !alt_on) || (shift_on && !ctrl_on && !alt_on)) { // ctrl/shift
				std::vector<Eigen::Vector3d> feature_points_save = feature_points;
				if (existing_edge_label && geodesic_path.size() >= 2) { geodesic_assign_vector(); }
				else { assign_vector(); }
				symmetry_assign_vector(feature_points_save);
                should_redraw = true;

            // seaming line
			} else if (alt_on && !shift_on && !ctrl_on) { // alt
                std::vector<Eigen::Vector3d> feature_points_save = feature_points;
                if (existing_edge_label && geodesic_path.size() >= 2) { geodesic_split_mesh(); }
                else { split_mesh(); }
                symmetry_split_mesh(feature_points_save);
                cut_along_seams();
                should_redraw = true;
            }
        }
    } 

    ctrl_on = false;
    alt_on = false;
    shift_on = false;
    feature_points.clear();
    feature_face_ids.clear();
    loop_feature_face_ids.clear();

    // clear geodesic data
    if (existing_edge_label) {
        existing_edge_label = false;
        geodesic_path.clear();
        std::vector<int>().swap(geodesic_path);
    }

    if (should_redraw) {
        interpolate_field();
        update_raw_field();
        update_visualization();
		save_ctrlz();
		should_redraw = false;
    }

	return false;
}

/////////////////////////////////////////////////////////////////////////////
void RemeshingMenu::assign_vector(int face_id, Eigen::Vector3d n) {
    face_vectors[face_id].assigned[drawing_mode] = true;
    // n is not normalized any more because we want to keep track of sizing information
    face_vectors[face_id].frame[drawing_mode] = n;
    if (ctrl_on) face_vectors[face_id].is_hard = true;
    if (shift_on) face_vectors[face_id].is_hard = false;
    should_redraw = true;
}

void RemeshingMenu::assign_vector() {
	// compute cutting edges
	std::vector<std::vector<int>> cutting_faces(F.rows(), std::vector<int>());
	std::vector<int> cutting_0_edges;
	std::vector<int> cutting_1_edges;
	std::vector<Eigen::Vector3d> cutting_points;
	CGAL_Mesh_Cutting(feature_points, mesh_edge_size * click_threshold,
		igl_tree, feature_face_ids, cutting_0_edges, cutting_1_edges, cutting_points, cutting_faces);

    // feature
    if (feature_face_ids[0] == feature_face_ids[feature_face_ids.size() - 1]) {
        if (feature_face_ids[0] == feature_face_ids[1]) {
            int face_id = feature_face_ids[0];

            Eigen::Vector3d vec = feature_points[feature_points.size() - 1] - feature_points[0];
            Eigen::Vector3d v_0 = V.row(F.row(face_id)[0]);
            Eigen::Vector3d v_1 = V.row(F.row(face_id)[1]);
            Eigen::Vector3d v_2 = V.row(F.row(face_id)[2]);
            Eigen::Vector3d n_0 = face_vectors[face_id].normal.cross(v_1 - v_0);
            Eigen::Vector3d n_1 = face_vectors[face_id].normal.cross(v_2 - v_1);
            Eigen::Vector3d n_2 = face_vectors[face_id].normal.cross(v_0 - v_2);

            double angle_0 = angle_between(n_0, vec);
            double angle_1 = angle_between(n_1, vec);
            double angle_2 = angle_between(n_2, vec);
            double angle_3 = angle_between(-n_0, vec);
            double angle_4 = angle_between(-n_1, vec);
            double angle_5 = angle_between(-n_2, vec);

            if (angle_0 < angle_1 && angle_0 < angle_2 && angle_0 < angle_3 && angle_0 < angle_4 && angle_0 < angle_5) {
                assign_vector(face_id, n_0);
            } 
            if (angle_1 < angle_0 && angle_1 < angle_2 && angle_1 < angle_3 && angle_1 < angle_4 && angle_1 < angle_5) {
                assign_vector(face_id, n_1); 
            }
            if (angle_2 < angle_0 && angle_2 < angle_1 && angle_2 < angle_3 && angle_2 < angle_4 && angle_2 < angle_5) {
                assign_vector(face_id, n_2);
            }
            if (angle_3 < angle_0 && angle_3 < angle_1 && angle_3 < angle_2 && angle_3 < angle_4 && angle_3 < angle_5) {
                assign_vector(face_id, -n_0);
            }
            if (angle_4 < angle_0 && angle_4 < angle_1 && angle_4 < angle_2 && angle_4 < angle_3 && angle_4 < angle_5) {
                assign_vector(face_id, -n_1);
            }
            if (angle_5 < angle_0 && angle_5 < angle_1 && angle_5 < angle_2 && angle_5 < angle_3 && angle_5 < angle_4) {
                assign_vector(face_id, -n_2);
            }
        }
    }

	for (int i = 0; i < cutting_faces.size(); i++) {
		if (cutting_faces[i].size() == 2) {
			int index_0 = cutting_faces[i][0];
			int index_1 = cutting_faces[i][1];
			assign_vector(i, cutting_points[index_1] - cutting_points[index_0]);
		}
	}

    if (!loop_feature_face_ids.empty()) {
        for (int i = 0; i < cutting_faces.size(); i++) {
            if (cutting_faces[i].size() == 1) {
                int index = cutting_faces[i][0];
                if (loop_feature_face_ids.front() == i) assign_vector(i, cutting_points[index] - feature_points.front());
                if (loop_feature_face_ids.back() == i) assign_vector(i, feature_points.back() - cutting_points[index]);
            }
        }
    }

	std::vector<std::vector<int>>().swap(cutting_faces);
}

std::unordered_set<int> RemeshingMenu::neighbor_faces(std::vector<std::unordered_set<int>>& igl_faces, int e_0, int e_1) {
    std::unordered_set<int> faces;
    for (int face_index : igl_faces[e_0]) {
        int index_0 = F.row(face_index)[0];
        int index_1 = F.row(face_index)[1];
        int index_2 = F.row(face_index)[2];
        if (index_0 == e_1 || index_1 == e_1 || index_2 == e_1) {
            faces.emplace(face_index);
        }
    }
    for (int face_index : igl_faces[e_1]) {
        int index_0 = F.row(face_index)[0];
        int index_1 = F.row(face_index)[1];
        int index_2 = F.row(face_index)[2];
        if (index_0 == e_0 || index_1 == e_0 || index_2 == e_0) {
            faces.emplace(face_index);
        }
    }
    return faces;
}

void RemeshingMenu::geodesic_assign_vector() {
    // point index => face
    for (int i = 1; i < geodesic_path.size(); i++) {
        int e_0 = geodesic_path[i - 1];
        int e_1 = geodesic_path[i];
        Eigen::Vector3d v_0 = V.row(e_0);
        Eigen::Vector3d v_1 = V.row(e_1);
        std::unordered_set<int> n_faces = neighbor_faces(igl_v_faces, e_0, e_1);
        for (int face_index : n_faces) {
            assign_vector(face_index, v_1 - v_0);
        }
    }
}

bool RemeshingMenu::same_line(
	const std::vector<Eigen::Vector3d> & feature_points_save, const std::vector<Eigen::Vector3d> & feature_points) {

	double d0 = (feature_points_save[0] - feature_points[feature_points.size() - 1]).norm();
	double d1 = (feature_points_save[feature_points_save.size() - 1] - feature_points[0]).norm();
	return (d0 + d1) < mesh_size * 0.15;
}

void RemeshingMenu::symmetry_assign_vector(std::vector<int> axes, int start, int end) {
	bool sym = true;
	for (int axis : axes) sym = sym && symmetrizer.has_vertex_symmetry(axis);
	if (sym) {
		for (int axis : axes) {
			start = symmetrizer.symmetric_vertex(start, axis);
			end = symmetrizer.symmetric_vertex(end, axis);
		}
		graph_dijkstra(graph_adj, start, end, geodesic_path);
		geodesic_assign_vector();
	}
}

void RemeshingMenu::symmetry_assign_vector(
	std::vector<int> axes, const std::vector<Eigen::Vector3d> & feature_points_save) {

	feature_points.clear();
	for (int i = 0; i < feature_points_save.size(); i++) {
		Eigen::Vector3d sym_v = feature_points_save[i];
		for (int axis : axes) {
			sym_v[axis] = sym_v[axis] + 2.0 * (mesh_center[axis] - sym_v[axis]);
		}
		feature_points.push_back(sym_v);
	}
	if (!same_line(feature_points_save, feature_points)) assign_vector();
}

void RemeshingMenu::symmetry_assign_vector(const std::vector<Eigen::Vector3d> & feature_points_save) {
	if (existing_edge_label && geodesic_path.size() >= 2) {
		//geodesic_path
		int start_index = geodesic_path[0];
		int end_index = geodesic_path[geodesic_path.size() - 1];

		//x
		if (symmetry_mode_yz && !symmetry_mode_xz && !symmetry_mode_xy) {
			symmetry_assign_vector({ 0 }, start_index, end_index);
		}
		//y
		if (symmetry_mode_xz && !symmetry_mode_yz && !symmetry_mode_xy) {
			symmetry_assign_vector({ 1 }, start_index, end_index);
		}
		//z
		if (symmetry_mode_xy && !symmetry_mode_yz && !symmetry_mode_xz) {
			symmetry_assign_vector({ 2 }, start_index, end_index);
		}

		//x y
		if (symmetry_mode_yz && symmetry_mode_xz && !symmetry_mode_xy) {
			//x
			symmetry_assign_vector({ 0 }, start_index, end_index);
			//y
			symmetry_assign_vector({ 1 }, start_index, end_index);
			//x y
			symmetry_assign_vector({ 0, 1 }, start_index, end_index);
		}
		//y z
		if (symmetry_mode_xz && symmetry_mode_xy && !symmetry_mode_yz) {
			//y
			symmetry_assign_vector({ 1 }, start_index, end_index);
			//z
			symmetry_assign_vector({ 2 }, start_index, end_index);
			//y z
			symmetry_assign_vector({ 1, 2 }, start_index, end_index);
		}
		//x z
		if (symmetry_mode_yz && symmetry_mode_xy && !symmetry_mode_xz) {
			//x
			symmetry_assign_vector({ 0 }, start_index, end_index);
			//z
			symmetry_assign_vector({ 2 }, start_index, end_index);
			//x z
			symmetry_assign_vector({ 0, 2 }, start_index, end_index);
		}

		//x y z
		if (symmetry_mode_yz && symmetry_mode_xy && symmetry_mode_xz) {
			//x
			symmetry_assign_vector({ 0 }, start_index, end_index);
			//y
			symmetry_assign_vector({ 1 }, start_index, end_index);
			//z
			symmetry_assign_vector({ 2 }, start_index, end_index);
			//x z
			symmetry_assign_vector({ 0, 2 }, start_index, end_index);
			//x y
			symmetry_assign_vector({ 0, 1 }, start_index, end_index);
			//z y
			symmetry_assign_vector({ 2, 1 }, start_index, end_index);
			//x z y
			symmetry_assign_vector({ 0, 2, 1 }, start_index, end_index);
		}

	} else {
		//x
		if (symmetry_mode_yz && !symmetry_mode_xz && !symmetry_mode_xy) {
			symmetry_assign_vector({ 0 }, feature_points_save);
		}
		//y
		if (symmetry_mode_xz && !symmetry_mode_yz && !symmetry_mode_xy) {
			symmetry_assign_vector({ 1 }, feature_points_save);
		}
		//z
		if (symmetry_mode_xy && !symmetry_mode_yz && !symmetry_mode_xz) {
			symmetry_assign_vector({ 2 }, feature_points_save);
		}

		//x y
		if (symmetry_mode_yz && symmetry_mode_xz && !symmetry_mode_xy) {
			//x
			symmetry_assign_vector({ 0 }, feature_points_save);
			//y
			symmetry_assign_vector({ 1 }, feature_points_save);
			//x y
			symmetry_assign_vector({ 0, 1 }, feature_points_save);
		}
		//y z
		if (symmetry_mode_xz && symmetry_mode_xy && !symmetry_mode_yz) {
			//y
			symmetry_assign_vector({ 1 }, feature_points_save);
			//z
			symmetry_assign_vector({ 2 }, feature_points_save);
			//y z
			symmetry_assign_vector({ 1, 2 }, feature_points_save);
		}
		//x z
		if (symmetry_mode_yz && symmetry_mode_xy && !symmetry_mode_xz) {
			//x
			symmetry_assign_vector({ 0 }, feature_points_save);
			//z
			symmetry_assign_vector({ 2 }, feature_points_save);
			//x z
			symmetry_assign_vector({ 0, 2 }, feature_points_save);
		}

		//x y z
		if (symmetry_mode_yz && symmetry_mode_xy && symmetry_mode_xz) {
			//x
			symmetry_assign_vector({ 0 }, feature_points_save);
			//y
			symmetry_assign_vector({ 1 }, feature_points_save);
			//z
			symmetry_assign_vector({ 2 }, feature_points_save);
			//x y
			symmetry_assign_vector({ 0, 1 }, feature_points_save);
			//x z
			symmetry_assign_vector({ 0, 2 }, feature_points_save);
			//y z
			symmetry_assign_vector({ 1, 2 }, feature_points_save);
			//x y z
			symmetry_assign_vector({ 0, 1, 2 }, feature_points_save);
		}
	}
}

void RemeshingMenu::split_mesh() {

	// compute cutting edges
	std::vector<std::vector<int>> cutting_faces(F.rows(), std::vector<int>());
	std::vector<int> cutting_0_edges;
	std::vector<int> cutting_1_edges;
	std::vector<Eigen::Vector3d> cutting_points;
	CGAL_Mesh_Cutting(feature_points, mesh_edge_size * click_threshold,
		igl_tree, feature_face_ids, cutting_0_edges, cutting_1_edges, cutting_points, cutting_faces);

    // remesh
    std::vector<Eigen::Vector3d> new_points;
    for (int i = 0; i < V.rows(); i++) {
        // 0 ~ V.rows-1
        new_points.push_back(V.row(i));
    }
    for (int i = 0; i < cutting_points.size(); i++) {
        // V.rows ~ V.rows+cutting_points.size()-1
        new_points.push_back(cutting_points[i]);
    }

    // new faces
    std::vector<Eigen::Vector3i> new_faces;
    std::vector<int> face_refs;
    for (int i = 0; i < cutting_faces.size(); i++) {
        if (cutting_faces[i].size() == 0) {
            face_refs.push_back(new_faces.size());
            face_refs.push_back(i);
            new_faces.push_back(F.row(i));
        }

        if (cutting_faces[i].size() == 1) {
            int v_index_0 = F.row(i)[0];
            int v_index_1 = F.row(i)[1];
            int v_index_2 = F.row(i)[2];

            int cutting_edge_index = cutting_faces[i][0];
            int end_0 = cutting_0_edges[cutting_edge_index];
            int end_1 = cutting_1_edges[cutting_edge_index];

            int insert_v_index = V.rows() + cutting_edge_index;

            int next_edge_index = -1;
            int existing_index_0 = -1;
            int existing_index_1 = -1;
            if ((end_0 == v_index_0 && end_1 == v_index_1) || (end_0 == v_index_1 && end_1 == v_index_0)) {
                // 0 index 2
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index, v_index_2));
                // index 1 2
                new_faces.push_back(Eigen::Vector3i(insert_v_index, v_index_1, v_index_2));
                next_edge_index = v_index_2;
                existing_index_0 = v_index_0;
                existing_index_1 = v_index_1;
            }
            if ((end_0 == v_index_0 && end_1 == v_index_2) || (end_0 == v_index_2 && end_1 == v_index_0)) {
                // 0 1 index
                new_faces.push_back(Eigen::Vector3i(v_index_0, v_index_1, insert_v_index));
                // index 1 2
                new_faces.push_back(Eigen::Vector3i(insert_v_index, v_index_1, v_index_2));
                next_edge_index = v_index_1;
                existing_index_0 = v_index_0;
                existing_index_1 = v_index_2;
            }
            if ((end_0 == v_index_1 && end_1 == v_index_2) || (end_0 == v_index_2 && end_1 == v_index_1)) {
                // 0 1 index
                new_faces.push_back(Eigen::Vector3i(v_index_0, v_index_1, insert_v_index));
                // 0 index 2
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index, v_index_2));
                next_edge_index = v_index_0;
                existing_index_0 = v_index_1;
                existing_index_1 = v_index_2;
            }

            // check the first point
            if (next_edge_index >= 0) {
                double distance = (feature_points[0] - V.row(next_edge_index).transpose()).norm();
                if (feature_face_ids[0] == i) {
                    if (distance < mesh_edge_size * click_threshold) {
                        split_edges.push_back({ insert_v_index, next_edge_index, face_vectors[i].normal });
                        split_existing_edges(existing_index_0, existing_index_1, insert_v_index);
                    }
                }
                if (feature_face_ids[feature_face_ids.size() - 1] == i) {
                    if (distance < mesh_edge_size * click_threshold) {
                        split_edges.push_back({ insert_v_index, next_edge_index, face_vectors[i].normal });
                        split_existing_edges(existing_index_0, existing_index_1, insert_v_index);
                    }
                }
            }
        }

        if (cutting_faces[i].size() == 2) {
            int v_index_0 = F.row(i)[0];
            int v_index_1 = F.row(i)[1];
            int v_index_2 = F.row(i)[2];

            int cutting_edge_index_0 = cutting_faces[i][0];
            int end_0_0 = cutting_0_edges[cutting_edge_index_0];
            int end_0_1 = cutting_1_edges[cutting_edge_index_0];

            int cutting_edge_index_1 = cutting_faces[i][1];
            int end_1_0 = cutting_0_edges[cutting_edge_index_1];
            int end_1_1 = cutting_1_edges[cutting_edge_index_1];

            int insert_v_index_0 = V.rows() + cutting_edge_index_0;
            int insert_v_index_1 = V.rows() + cutting_edge_index_1;

            // get share point index
            int share_point_index = -1;
            int point_index_0 = -1;
            int point_index_1 = -1;

            if (end_0_0 == end_1_0) {
                share_point_index = end_0_0;
                point_index_0 = end_0_1;
                point_index_1 = end_1_1;
            }

            if (end_0_1 == end_1_1) {
                share_point_index = end_0_1;
                point_index_0 = end_0_0;
                point_index_1 = end_1_0;
            }

            if (end_0_0 == end_1_1) {
                share_point_index = end_0_0;
                point_index_0 = end_0_1;
                point_index_1 = end_1_0;
            }

            if (end_0_1 == end_1_0) {
                share_point_index = end_0_1;
                point_index_0 = end_0_0;
                point_index_1 = end_1_1;
            }

            //A
            if (share_point_index == v_index_0 && point_index_0 == v_index_2 && point_index_1 == v_index_1) {
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_1, insert_v_index_0));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_1, v_index_1, insert_v_index_0));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_0, v_index_1, v_index_2));

                split_existing_edges(v_index_0, v_index_1, insert_v_index_1);
                split_existing_edges(v_index_0, v_index_2, insert_v_index_0);
            }

            //B
            if (share_point_index == v_index_0 && point_index_0 == v_index_1 && point_index_1 == v_index_2) {
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_0, insert_v_index_1));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_0, v_index_1, insert_v_index_1));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_1, v_index_1, v_index_2));

                split_existing_edges(v_index_0, v_index_1, insert_v_index_0);
                split_existing_edges(v_index_0, v_index_2, insert_v_index_1);
            }

            //C
            if (share_point_index == v_index_2 && point_index_0 == v_index_1 && point_index_1 == v_index_0) {
                new_faces.push_back(Eigen::Vector3i(v_index_0, v_index_1, insert_v_index_0));
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_0, insert_v_index_1));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_1, insert_v_index_0, v_index_2));

                split_existing_edges(v_index_0, v_index_2, insert_v_index_1);
                split_existing_edges(v_index_1, v_index_2, insert_v_index_0);
            }

            //D
            if (share_point_index == v_index_2 && point_index_0 == v_index_0 && point_index_1 == v_index_1) {
                new_faces.push_back(Eigen::Vector3i(v_index_0, v_index_1, insert_v_index_1));
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_1, insert_v_index_0));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_0, insert_v_index_1, v_index_2));

                split_existing_edges(v_index_0, v_index_2, insert_v_index_0);
                split_existing_edges(v_index_1, v_index_2, insert_v_index_1);
            }

            //E
            if (share_point_index == v_index_1 && point_index_0 == v_index_0 && point_index_1 == v_index_2) {
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_0, insert_v_index_1));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_0, v_index_1, insert_v_index_1));
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_1, v_index_2));

                split_existing_edges(v_index_0, v_index_1, insert_v_index_0);
                split_existing_edges(v_index_1, v_index_2, insert_v_index_1);
            }

            //F
            if (share_point_index == v_index_1 && point_index_0 == v_index_2 && point_index_1 == v_index_0) {
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_1, insert_v_index_0));
                new_faces.push_back(Eigen::Vector3i(insert_v_index_1, v_index_1, insert_v_index_0));
                new_faces.push_back(Eigen::Vector3i(v_index_0, insert_v_index_0, v_index_2));

                split_existing_edges(v_index_0, v_index_1, insert_v_index_1);
                split_existing_edges(v_index_1, v_index_2, insert_v_index_0);
            }

            split_edges.push_back({ insert_v_index_0, insert_v_index_1, face_vectors[i].normal });
        }
    }

    ///////////////////////////////////////

    Eigen::MatrixX3d newV;
    newV.resize(new_points.size(), 3);
    for (int i = 0; i < new_points.size(); i++) { newV.row(i) = new_points[i]; }
    V = newV;

    Eigen::MatrixX3i newF;
    newF.resize(new_faces.size(), 3);
    for (int i = 0; i < new_faces.size(); i++) { newF.row(i) = new_faces[i]; }
    F = newF;

    setup_mesh();
    update_polyhedron_tree(face_refs);

    ///////////////////////////////////////

    // update face vectors
    std::vector<FaceVector> new_face_vectors;
    for (int i = 0; i < F.rows(); i++) {
        FaceVector fv;
        fv.face_id = i;
        Eigen::Vector3d v_0 = V.row(F.row(i)[0]);
        Eigen::Vector3d v_1 = V.row(F.row(i)[1]);
        Eigen::Vector3d v_2 = V.row(F.row(i)[2]);
        fv.center = (v_0 + v_1 + v_2) / 3.0;
        fv.normal = (v_1 - v_0).cross(v_2 - v_0).normalized();
        fv.assigned = { false, false };
        fv.frame = { Eigen::Vector3d(), Eigen::Vector3d() };
        new_face_vectors.push_back(fv);
    }
    for (int i = 0; i < face_refs.size(); i = i + 2) {
        int new_face_index = face_refs[i];
        int old_face_index = face_refs[i + 1];
        new_face_vectors[new_face_index] = face_vectors[old_face_index];
        new_face_vectors[new_face_index].face_id = new_face_index;
    }
    face_vectors.clear();
    face_vectors = new_face_vectors;
    new_face_vectors.clear();
}

void RemeshingMenu::geodesic_split_mesh() {
	// point index => face
	for (int i = 1; i < geodesic_path.size(); i++) {
		int e_0 = geodesic_path[i - 1];
		int e_1 = geodesic_path[i];
		std::unordered_set<int> n_faces = neighbor_faces(igl_v_faces, e_0, e_1);
		split_edges.push_back({ e_0, e_1, face_vectors[*(n_faces.begin())].normal });
	}
}

void RemeshingMenu::split_existing_edges(int index_0, int index_1, int insert_index) {
	int index = -1;
	for (int i = 0; i < split_edges.size(); i++) {
		if ((split_edges[i].index_0 == index_0 && split_edges[i].index_1 == index_1) ||
			(split_edges[i].index_1 == index_0 && split_edges[i].index_0 == index_1)) {
			index = i;
			break;
		}
	}
	if (index >= 0) { // perform the split
		SplitEdge edge = split_edges[index];
		split_edges.erase(split_edges.begin() + index);
		split_edges.push_back({ index_0, insert_index, edge.normal });
		split_edges.push_back({ index_1, insert_index, edge.normal });
	}
}

void RemeshingMenu::symmetry_split_mesh(std::vector<int> axes, int start, int end) {
	bool sym = true;
	for (int axis : axes) sym = sym && symmetrizer.has_vertex_symmetry(axis);
	if (sym) {
		for (int axis : axes) {
			start = symmetrizer.symmetric_vertex(start, axis);
			end = symmetrizer.symmetric_vertex(end, axis);
		}
		graph_dijkstra(graph_adj, start, end, geodesic_path);
		geodesic_split_mesh();
	}
}

void RemeshingMenu::symmetry_split_mesh(
	std::vector<int> axes, const std::vector<Eigen::Vector3d> & feature_points_save) {

	feature_points.clear();
	for (int i = 0; i < feature_points_save.size(); i++) {
		Eigen::Vector3d sym_v = feature_points_save[i];
		for (int axis : axes) {
			sym_v[axis] = sym_v[axis] + 2.0 * (mesh_center[axis] - sym_v[axis]);
		}
		feature_points.push_back(sym_v);
	}
	if (!same_line(feature_points_save, feature_points)) split_mesh();
}

void RemeshingMenu::symmetry_split_mesh(const std::vector<Eigen::Vector3d> & feature_points_save) {

	if (existing_edge_label && geodesic_path.size() >= 2) {
		//geodesic_path
		int start_index = geodesic_path[0];
		int end_index = geodesic_path[geodesic_path.size() - 1];

		//x
		if (symmetry_mode_yz && !symmetry_mode_xz && !symmetry_mode_xy) {
			symmetry_split_mesh({ 0 }, start_index, end_index);
		}
		//y
		if (symmetry_mode_xz && !symmetry_mode_yz && !symmetry_mode_xy) {
			symmetry_split_mesh({ 1 }, start_index, end_index);
		}
		//z
		if (symmetry_mode_xy && !symmetry_mode_yz && !symmetry_mode_xz) {
			symmetry_split_mesh({ 2 }, start_index, end_index);
		}

		//x y
		if (symmetry_mode_yz && symmetry_mode_xz && !symmetry_mode_xy) {
			//x
			symmetry_split_mesh({ 0 }, start_index, end_index);
			//y
			symmetry_split_mesh({ 1 }, start_index, end_index);
			//x y
			symmetry_split_mesh({ 0, 1 }, start_index, end_index);
		}
		//y z
		if (symmetry_mode_xz && symmetry_mode_xy && !symmetry_mode_yz) {
			//y
			symmetry_split_mesh({ 1 }, start_index, end_index);
			//z
			symmetry_split_mesh({ 2 }, start_index, end_index);
			//y z
			symmetry_split_mesh({ 1, 2 }, start_index, end_index);
		}
		//x z
		if (symmetry_mode_yz && symmetry_mode_xy && !symmetry_mode_xz) {
			//x
			symmetry_split_mesh({ 0 }, start_index, end_index);
			//z
			symmetry_split_mesh({ 2 }, start_index, end_index);
			//x z
			symmetry_split_mesh({ 0, 2 }, start_index, end_index);
		}

		//x y z
		if (symmetry_mode_yz && symmetry_mode_xy && symmetry_mode_xz) {
			//x
			symmetry_split_mesh({ 0 }, start_index, end_index);
			//y
			symmetry_split_mesh({ 1 }, start_index, end_index);
			//z
			symmetry_split_mesh({ 2 }, start_index, end_index);
			//x z
			symmetry_split_mesh({ 0, 2 }, start_index, end_index);
			//x y
			symmetry_split_mesh({ 0, 1 }, start_index, end_index);
			//y z
			symmetry_split_mesh({ 2, 1 }, start_index, end_index);
			//x z y
			symmetry_split_mesh({ 0, 2, 1 }, start_index, end_index);
		}

	} else {
		//x
		if (symmetry_mode_yz && !symmetry_mode_xz && !symmetry_mode_xy) {
			symmetry_split_mesh({ 0 }, feature_points_save);
		}
		//y
		if (symmetry_mode_xz && !symmetry_mode_yz && !symmetry_mode_xy) {
			symmetry_split_mesh({ 1 }, feature_points_save);
		}
		//z
		if (symmetry_mode_xy && !symmetry_mode_yz && !symmetry_mode_xz) {
			symmetry_split_mesh({ 2 }, feature_points_save);
		}

		//x y
		if (symmetry_mode_yz && symmetry_mode_xz && !symmetry_mode_xy) {
			//x
			symmetry_split_mesh({ 0 }, feature_points_save);
			//y
			symmetry_split_mesh({ 1 }, feature_points_save);
			//x y
			symmetry_split_mesh({ 0, 1 }, feature_points_save);
		}
		//y z
		if (symmetry_mode_xz && symmetry_mode_xy && !symmetry_mode_yz) {
			//y
			symmetry_split_mesh({ 1 }, feature_points_save);
			//z
			symmetry_split_mesh({ 2 }, feature_points_save);
			//y z
			symmetry_split_mesh({ 1, 2 }, feature_points_save);
		}
		//x z
		if (symmetry_mode_yz && symmetry_mode_xy && !symmetry_mode_xz) {
			//x
			symmetry_split_mesh({ 0 }, feature_points_save);
			//z
			symmetry_split_mesh({ 2 }, feature_points_save);
			//x z
			symmetry_split_mesh({ 0, 2 }, feature_points_save);
		}

		//x y z
		if (symmetry_mode_yz && symmetry_mode_xy && symmetry_mode_xz) {
			//x
			symmetry_split_mesh({ 0 }, feature_points_save);
			//z
			symmetry_split_mesh({ 2 }, feature_points_save);
			//x z
			symmetry_split_mesh({ 0, 2 }, feature_points_save);
			//y
			symmetry_split_mesh({ 1 }, feature_points_save);
			//x y
			symmetry_split_mesh({ 0, 1 }, feature_points_save);
			//z y
			symmetry_split_mesh({ 1, 2 }, feature_points_save);
			//x z y
			symmetry_split_mesh({ 0, 1, 2 }, feature_points_save);
		}
	}
}

void RemeshingMenu::cut_along_seams() {
    std::vector<std::vector<int>> VF, VI;
    igl::vertex_triangle_adjacency(V.rows(), F, VF, VI);

    Eigen::MatrixXi seams(F.rows(), 3);
    seams.setZero();
    for (const SplitEdge& se : split_edges) {
        int v0 = se.index_0;
        int v1 = se.index_1;
        for (int i = 0; i < VF[v0].size(); ++i) {
            int f = VF[v0][i];
            int idx = VI[v0][i];
            assert(F(f, idx) == v0);
            if (F(f, (idx + 1) % 3) == v1) {
                seams(f, idx) = 1;
                continue;
            }
        }
    }

    Eigen::MatrixXd NV;
    Eigen::MatrixXi NF;
    igl::cut_mesh(V, F, seams, NV, NF);
    V = NV;
    F = NF;
    setup_mesh();
    update_polyhedron_tree();
    reset_face_vectors(); // TODO: instead of resetting, keep track of old face vectors
    setup_boundary();
    interpolate_field();
    setup_basis_cycles();
    update_visualization();
}

///////////////////////////////// DRAW IMPLS /////////////////////////////////
//TODO: remove unnecessary drawing (use underlying_loop_edges to debug?)
void RemeshingMenu::update_drawing() {
#ifdef HAISEN1
	for (auto& node : loop_gi_nodes) {
		draw_a_segment(V.row(node.g_n_0), V.row(node.g_n_1), 2);
	}
	for (auto& edge : loop_gi_edges) {
		draw_a_segment(loop_gi_nodes[edge.n_0].m, loop_gi_nodes[edge.n_1].m, 2);
	}
#endif

    // axis
    if (show_axis) {
        draw_a_segment(mesh_center, mesh_center + Eigen::Vector3d(1.0, 0.0, 0.0) * mesh_size, 0);
        draw_a_segment(mesh_center, mesh_center + Eigen::Vector3d(0.0, 1.0, 0.0) * mesh_size, 1);
        draw_a_segment(mesh_center, mesh_center + Eigen::Vector3d(0.0, 0.0, 1.0) * mesh_size, 2);
    }

	// face vectors
    for (int i = 0; i < face_vectors.size(); i++) {
        if (face_vectors[i].assigned[0]) {
            double mesh_scale = mesh_size * 0.02;  // 0.002
            Eigen::Vector3d nudge = face_vectors[i].normal * mesh_size * 0.001;
            // frame 1
            Eigen::Vector3d s = face_vectors[i].center - face_vectors[i].frame[0].normalized() * mesh_scale;
            Eigen::Vector3d t = face_vectors[i].center + face_vectors[i].frame[0].normalized() * mesh_scale;
            s += nudge; t += nudge;
            draw_a_segment(s, t, 2);
            // arrow 1
            Eigen::Vector3d base = face_vectors[i].frame[0].cross(face_vectors[i].normal);
            Eigen::Vector3d arrow_base = face_vectors[i].center + face_vectors[i].frame[0].normalized() * mesh_size * 0.02 * 0.8;
            Eigen::Vector3d arrow_point_0 = arrow_base - base.normalized() * mesh_scale * (1.0 - 0.8);
            Eigen::Vector3d arrow_point_1 = arrow_base + base.normalized() * mesh_scale * (1.0 - 0.8);
            arrow_point_0 += nudge; arrow_point_1 += nudge;
            if (face_vectors[i].is_hard) {
                draw_a_segment(t, arrow_point_0, 0); draw_a_segment(t, arrow_point_1, 0);
            } else {
                draw_a_segment(t, arrow_point_0, 2); draw_a_segment(t, arrow_point_1, 2);
            }
        }
        if (face_vectors[i].assigned[1]) {
            double mesh_scale = mesh_size * 0.02;  // 0.002
            Eigen::Vector3d nudge = face_vectors[i].normal * mesh_size * 0.001;
            // frame 2
            Eigen::Vector3d s = face_vectors[i].center - face_vectors[i].frame[1].normalized() * mesh_scale;
            Eigen::Vector3d t = face_vectors[i].center + face_vectors[i].frame[1].normalized() * mesh_scale;
            s += nudge; t += nudge;
            draw_a_segment(s, t, 3);
            // arrow 2
            Eigen::Vector3d base = face_vectors[i].frame[1].cross(face_vectors[i].normal);
            Eigen::Vector3d arrow_base = face_vectors[i].center + face_vectors[i].frame[1].normalized() * mesh_size * 0.02 * 0.8;
            Eigen::Vector3d arrow_point_0 = arrow_base - base.normalized() * mesh_scale * (1.0 - 0.8);
            Eigen::Vector3d arrow_point_1 = arrow_base + base.normalized() * mesh_scale * (1.0 - 0.8);
            arrow_point_0 += nudge; arrow_point_1 += nudge;
            if (face_vectors[i].is_hard) {
                draw_a_segment(t, arrow_point_0, 0); draw_a_segment(t, arrow_point_1, 0);
            } else {
                draw_a_segment(t, arrow_point_0, 2); draw_a_segment(t, arrow_point_1, 2);
            }
        }
    }

	// draw split edges
    for (int i = 0; i < split_edges.size(); i++) {
        Eigen::Vector3d v_0 = V.row(split_edges[i].index_0);
        Eigen::Vector3d v_1 = V.row(split_edges[i].index_1);
        Eigen::Vector3d s = v_0 + split_edges[i].normal * mesh_size * 0.001;
        Eigen::Vector3d t = v_1 + split_edges[i].normal * mesh_size * 0.001;
        draw_a_segment(s, t, 1);
    }

	if (geodesic_label) { draw_a_point(geodesic_point, 0); }

    if (has_direction_field) { draw_direction_field(); }

	// loop's start & end points
	if (loop_start_index >= 0) 
		draw_a_point(loop_gi_nodes[loop_start_index].m, 1, 0.01);
	if (loop_end_index >= 0) 
		draw_a_point(loop_gi_nodes[loop_end_index].m, 2, 0.01);

	// loop path
	if (loop_path.size() >= 2) 
		draw_segments(loop_path,1,0.01);

	draw_points(loop_points, 1, 0.00);
	draw_points(loop_de_points, 0, 0.00);

	for (auto &line: loop_polylines)
		draw_segments(line, 0, 0.00);
	for (auto& line : loop_update_polylines)
		draw_segments(line, 2, 0.00);
}

void RemeshingMenu::draw_direction_field() {
    // Plot N-Rosy Mesh
    const Eigen::MatrixXd& PD1 = direction_field[1]; // draw only the wale field
    Eigen::MatrixXd Y(F.rows() * rosy, 3);
    for (int i = 0; i < F.rows(); ++i) {
        double x = PD1.row(i) * B1.row(i).transpose();
        double y = PD1.row(i) * B2.row(i).transpose();
        double angle = atan2(y, x);
        for (int j = 0; j < rosy; ++j) {
            double anglej = angle + 2 * igl::PI * (double) j / (double) rosy;
            Y.row(i * rosy + j) = cos(anglej) * B1.row(i) + sin(anglej) * B2.row(i);
        }
    }

    Eigen::MatrixXd Be(B.rows() * rosy, 3);
    for (int i = 0; i < B.rows(); ++i) {
        for (int j = 0; j < rosy; ++j) {
            Be.row(i * rosy + j) = B.row(i);
        }
    }

	Eigen::MatrixX3d edge_colors(Be.rows(), 3);
	for (int i = 0; i < Be.rows(); ++i) {
		edge_colors.row(i) = i % rosy == 0 ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(.7, .7, .7);
	}

    viewer->data().add_edges(Be, Be + Y * (global_scale / rosy), edge_colors);
}

//////////////// MAIN VIEW SWITCHING LOGIC ////////////////
void RemeshingMenu::set_mesh_overlays(const int mesh_id, const bool wireframe, const bool overlay, const bool fill) {
    viewer->data(mesh_id).show_lines = wireframe;
    viewer->data(mesh_id).show_faces = fill;
    viewer->data(mesh_id).show_overlay = overlay;
}

void RemeshingMenu::stylize_tri_mesh(const Eigen::MatrixXd& colors) {
    viewer->data().clear();
    viewer->data().set_mesh(V, F);
    if (has_integer_grid) {
        if (miq_mode == MIQMode::CROSS) {
            viewer->data().set_uv(V_uv, F_uv);
        } else {
            viewer->data().clear();
            viewer->data().set_mesh(VMeshCut, FMeshCut);
            viewer->data().set_uv(cutUV, FMeshCut);
        }
        viewer->data().set_texture(texture_R, texture_B, texture_G);
    }
    viewer->data().show_texture = has_integer_grid;
    viewer->data().set_colors(colors);
    set_mesh_overlays(viewer->selected_data_index, !has_integer_grid);
}

void RemeshingMenu::stylize_quad_mesh(const Eigen::MatrixXd& colors) {
    Eigen::MatrixX3d verts = quad_mesh.V;
    Eigen::MatrixX3i faces = quad_mesh.F_t;
    Eigen::MatrixXd TC(3, 2);
    TC.row(0) = Eigen::Vector2d(0, 0);
    TC.row(1) = Eigen::Vector2d(1, 0);
    TC.row(2) = Eigen::Vector2d(0.5, 0.5);
    Eigen::MatrixXi TUV(faces.rows(), 3);
    for (int i = 0; i < TUV.rows(); ++i) {
        TUV.row(i) = Eigen::Vector3i(0, 1, 2);
    }

    viewer->data_list[4].clear();
    viewer->data_list[4].set_mesh(verts, faces);
    viewer->data_list[4].set_uv(TC, TUV);
    viewer->data_list[4].set_colors(colors);
    viewer->data_list[4].set_texture(texture_R, texture_B, texture_G);
    viewer->data_list[4].show_texture = true;

    if (split_edges.empty()) return;

    igl::AABB<Eigen::MatrixXd, 3> aabb_tree;
    aabb_tree.init(quad_mesh.V, quad_mesh.F_t);

    for (const SplitEdge& se : split_edges) {
        Eigen::Vector3d v0 = V.row(se.index_0);
        Eigen::Vector3d v1 = V.row(se.index_1);
        Eigen::Vector3d edge = v1 - v0;
        Eigen::Vector3d nudge_dir = edge.cross(se.normal);

        Eigen::Vector3d nudged_midpoint = (v0 + v1) / 2. + 0.001 * mesh_size * nudge_dir;
        int fid;
        Eigen::RowVector3d C;
        aabb_tree.squared_distance(quad_mesh.V, quad_mesh.F_t, nudged_midpoint, fid, C);

        Eigen::Vector3d qv0 = quad_mesh.V.row(quad_mesh.side_u(fid));
        qv0 += se.normal * mesh_size * 0.005;
        Eigen::Vector3d qv1 = quad_mesh.V.row(quad_mesh.side_v(fid));
        qv1 += se.normal * mesh_size * 0.005;
        draw_a_segment(qv0, qv1, 1, -1., 4);
    }
}

void RemeshingMenu::update_visualization(unsigned char key) {
    for (const igl::opengl::ViewerData& data : viewer->data_list) {
        set_mesh_overlays(data.id, false, false, false);
    }

    switch (viewing_mode) {
    case ViewingMode::MESH_SING:
    {
        if (key == 'B' || key == 'G') {
            CMesh = directional::default_mesh_color().replicate(F.rows(), 1);
            for (int i = 0; i < cycleFaces[currCycle].size(); i++)
                CMesh.row(cycleFaces[currCycle][i]) << directional::selected_face_color();
            stylize_tri_mesh(CMesh);

        } else if (key == 'R') {
            Eigen::MatrixXd linfColors;
            Eigen::VectorXd squaredLinf = linf.array().square();
            igl::jet(squaredLinf, squaredLinf.minCoeff(), squaredLinf.maxCoeff(), linfColors);
            //igl::jet(squaredLinf, 0, 1, linfColors);
            CMesh = directional::default_mesh_color().replicate(F.rows(), 1);
            for (int c = 0; c < basisCycles.rows(); ++c) {
                for (int i = 0; i < cycleFaces[c].size(); i++) {
                    CMesh.row(cycleFaces[c][i]) = linfColors.row(c);
                }
            }
            stylize_tri_mesh(CMesh);

        } else {
            // raw field mesh
            directional::glyph_lines_raw(
                V, F, rawField, directional::default_glyph_color(),
                VField, FField, CField, 1.5);
            viewer->data_list[1].clear();
            viewer->data_list[1].set_mesh(VField, FField);
            viewer->data_list[1].set_colors(CField);
            set_mesh_overlays(1, false);
            // singularity mesh
            directional::singularity_spheres(
                V, F, N, singVertices, singIndices,
                VSings, FSings, CSings, 1.5);
            viewer->data_list[2].clear();
            viewer->data_list[2].set_mesh(VSings, FSings);
            viewer->data_list[2].set_colors(CSings);
            set_mesh_overlays(2, false);

            stylize_tri_mesh(directional::default_mesh_color());
        }
        break;
    }

    case ViewingMode::MESH_CURL:
    {
        if (!has_curl) { init_curl(); }
        Eigen::VectorXd currCurl = AE2F * curl;
        Eigen::MatrixXd curlColors;
        igl::jet(currCurl, 0.0, curlMaxOrig, curlColors);
        stylize_tri_mesh(curlColors);
        break;
    }

    case ViewingMode::MESH_FIELD:
    {
        if (!has_curl) { init_curl(); }
        // field mesh
        directional::glyph_lines_raw(
            V, F, combedField, directional::indexed_glyph_colors(combedField),
            VField, FField, CField, 2.0);
        viewer->data_list[1].clear();
        viewer->data_list[1].set_mesh(VField, FField);
        viewer->data_list[1].set_colors(CField);
        set_mesh_overlays(1, false);

        // singularity mesh
        directional::singularity_spheres(
            V, F, rosy, curlSingVertices, curlSingIndices,
            VSings, FSings, CSings, 1.5);
        viewer->data_list[2].clear();
        viewer->data_list[2].set_mesh(VSings, FSings);
        viewer->data_list[2].set_colors(CSings);
        set_mesh_overlays(2, false);

        // seam mesh
        directional::seam_lines(
            V, F, EV, combedMatching,
            VSeams, FSeams, CSeams, 2.0);
        viewer->data_list[3].clear();
        viewer->data_list[3].set_mesh(VSeams, FSeams);
        viewer->data_list[3].set_colors(CSeams);
        set_mesh_overlays(3, false);
    }

    case ViewingMode::MESH_ONLY:
    {
        stylize_tri_mesh(directional::default_mesh_color());
        if (viewing_mode != ViewingMode::MESH_FIELD) {
            update_drawing();
        }
        break;
    }

    case ViewingMode::MESH_QUAD:
    case ViewingMode::QUAD_ONLY:
    {
        stylize_quad_mesh(directional::default_mesh_color());
        set_mesh_overlays(4, false);
        if (viewing_mode == ViewingMode::MESH_QUAD) {
            set_mesh_overlays(viewer->selected_data_index);
        }
        break;
    }
    }
}

/////////////////////////////////////////////////////////////////////////////
void RemeshingMenu::update_polyhedron_tree(const std::vector<int>& face_refs) {
	///////////////////////////////////////
	igl_polyhedron.clear();
	if (!igl::copyleft::cgal::mesh_to_polyhedron(V, F, igl_polyhedron)) {
		std::cerr << "Cannot build CGAL polyhedron; ignoring...\n";
	}
	CGAL::set_halfedgeds_items_id(igl_polyhedron);
	igl_tree.clear();
	igl_tree.insert(faces(igl_polyhedron).first, faces(igl_polyhedron).second, igl_polyhedron);
	igl_tree.accelerate_distance_queries();
	igl_v_faces.clear();
	std::vector<std::unordered_set<int>>().swap(igl_v_faces);
    igl_v_faces = std::vector<std::unordered_set<int>>(V.rows(), std::unordered_set<int>());
    for (int i = 0; i < F.rows(); i++) {
        int index_0 = F.row(i)[0];
        int index_1 = F.row(i)[1];
        int index_2 = F.row(i)[2];
        igl_v_faces[index_0].emplace(i);
        igl_v_faces[index_1].emplace(i);
        igl_v_faces[index_2].emplace(i);
    }
	///////////////////////////////////////
	std::vector<std::vector<double>>().swap(graph_adj);
    graph_adj = std::vector<std::vector<double>>(V.rows(), std::vector<double>(V.rows(), 0.0));
	Eigen::MatrixXi E;
    igl::edges(F, E);
	for (int i = 0; i < E.rows(); i++) {
		int index_0 = E.row(i)[0];
		int index_1 = E.row(i)[1];
        Eigen::Vector3d e_0 = V.row(index_0);
        Eigen::Vector3d e_1 = V.row(index_1);
		double d = (e_0 - e_1).norm();
		graph_adj[index_0][index_1] = d;
		graph_adj[index_1][index_0] = d;
	}
	///////////////////////////////////////
    if (!face_refs.empty()) {
        std::vector<FaceVector> new_face_vectors(viewer->data().F.rows(), FaceVector());
        for (int i = 0; i < face_refs.size(); i = i + 2) {
            int new_face_index = face_refs[i];
            int old_face_index = face_refs[i + 1];
            new_face_vectors[new_face_index] = face_vectors[old_face_index];
            new_face_vectors[new_face_index].face_id = new_face_index;
        }
        for (int i = 0; i < viewer->data().F.rows(); i++) {
            auto& fv = new_face_vectors[i];
            if (fv.face_id < 0) {
                fv.face_id = i;
                Eigen::Vector3d v_0 = viewer->data().V.row(viewer->data().F.row(i)[0]);
                Eigen::Vector3d v_1 = viewer->data().V.row(viewer->data().F.row(i)[1]);
                Eigen::Vector3d v_2 = viewer->data().V.row(viewer->data().F.row(i)[2]);
                fv.center = (v_0 + v_1 + v_2) / 3.0;
                fv.normal = (v_1 - v_0).cross(v_2 - v_0).normalized();
            }
        }
        face_vectors = new_face_vectors;
    }
    ///////////////////////////////////////
	update_loop_graph();
}

void RemeshingMenu::apply_subdivision() {
    Eigen::MatrixXd NV;
    Eigen::MatrixXi NF;
    igl::loop(V, F, NV, NF);
    V = NV;
    F = NF;
    setup_mesh();
    update_polyhedron_tree();
    reset_face_vectors();
    setup_boundary();
    interpolate_field();
    setup_basis_cycles();
    update_visualization();
}

void RemeshingMenu::construct_half_edge(std::vector<int>& half_edges) {
    std::vector<std::unordered_set<int>> v_faces(V.rows(), std::unordered_set<int>());
    if (igl_v_faces.empty()) {
        for (int i = 0; i < F.rows(); i++) {
            int index_0 = F.row(i)[0];
            int index_1 = F.row(i)[1];
            int index_2 = F.row(i)[2];
            v_faces[index_0].emplace(i);
            v_faces[index_1].emplace(i);
            v_faces[index_2].emplace(i);
        }
    } else { v_faces = igl_v_faces; }
    for (const SplitEdge& se : split_edges) {
        std::unordered_set<int> face_ids = v_faces[se.index_0];
        face_ids.insert(v_faces[se.index_1].begin(), v_faces[se.index_1].end());
        for (const int face_index : face_ids) {
            int tri_0 = F.row(face_index)[0];
            int tri_1 = F.row(face_index)[1];
            int tri_2 = F.row(face_index)[2];
            if ((tri_0 == se.index_0 && tri_1 == se.index_1) || (tri_1 == se.index_0 && tri_0 == se.index_1)) {
                half_edges.push_back(face_index * 3);
            }
            if ((tri_1 == se.index_0 && tri_2 == se.index_1) || (tri_2 == se.index_0 && tri_1 == se.index_1)) {
                half_edges.push_back(face_index * 3 + 1);
            }
            if ((tri_2 == se.index_0 && tri_0 == se.index_1) || (tri_0 == se.index_0 && tri_2 == se.index_1)) {
                half_edges.push_back(face_index * 3 + 2);
            }
        }
    }
}

void RemeshingMenu::get_mesh_information() {
    Eigen::Vector3d minimal_vector = V.row(0);
    Eigen::Vector3d maximal_vector = V.row(0);

    for (int i = 0; i < V.rows(); i++) {
        mesh_center[0] += V.row(i).x();
        mesh_center[1] += V.row(i).y();
        mesh_center[2] += V.row(i).z();
        minimal_vector[0] = std::min(minimal_vector[0], V.row(i).x());
        minimal_vector[1] = std::min(minimal_vector[1], V.row(i).y());
        minimal_vector[2] = std::min(minimal_vector[2], V.row(i).z());
        maximal_vector[0] = std::max(maximal_vector[0], V.row(i).x());
        maximal_vector[1] = std::max(maximal_vector[1], V.row(i).y());
        maximal_vector[2] = std::max(maximal_vector[2], V.row(i).z());
    }

    double total_d = 0.0;
    for (int i = 0; i < F.rows(); i++) {
        Eigen::Vector3d v_0 = V.row(F(i, 0));
        Eigen::Vector3d v_1 = V.row(F(i, 1));
        Eigen::Vector3d v_2 = V.row(F(i, 2));
        double d0 = (v_0 - v_1).norm();
        double d1 = (v_1 - v_2).norm();
        double d2 = (v_2 - v_0).norm();
        total_d += (d0 + d1 + d2);
    }
    mesh_edge_size = total_d / (F.rows() * 3.0);

    mesh_size = 0.0;
    mesh_size += maximal_vector[0] - minimal_vector[0];
    mesh_size += maximal_vector[1] - minimal_vector[1];
    mesh_size += maximal_vector[2] - minimal_vector[2];
    mesh_size /= 3.0;

    mesh_center[0] = (maximal_vector[0] + minimal_vector[0]) / 2.0;
    mesh_center[1] = (maximal_vector[1] + minimal_vector[1]) / 2.0;
    mesh_center[2] = (maximal_vector[2] + minimal_vector[2]) / 2.0;
}

void RemeshingMenu::draw_a_segment(
    const Eigen::Vector3d v0, const Eigen::Vector3d v1,
    const int color_index, const double face_dis, const int data_index) {

    Eigen::MatrixXd vec0;
	vec0.resize(1, 3);
	vec0.row(0) = v0;
	Eigen::MatrixXd vec1;
	vec1.resize(1, 3);
	vec1.row(0) = v1;

	if (face_dis > 0.) {
		vec0.row(0)= v0 + face_vectors[CGAL_Closest_Face(igl_tree, v0)].normal * face_dis;
		vec1.row(0) = v1 + face_vectors[CGAL_Closest_Face(igl_tree, v1)].normal * face_dis;
	}

    Eigen::RowVector3d color;
    switch (color_index) {
    case 0:
        color = { 0.8, 0.2, 0.2 }; break; // red
    case 1:
        color = { 0.2, 0.8, 0.2 }; break; // green
    case 2:
        color = { 0.2, 0.2, 0.8 }; break; // blue
    case 3:
        color = { 0.9, 0.5, 0.1 }; break; // orange
    }
    viewer->data_list[data_index].add_edges(vec0, vec1, color);
}

void RemeshingMenu::draw_segments(const std::vector<Eigen::Vector3d>& segments, const int color_index, const double face_dis) {
	if (segments.size() >= 2) {
		for (int i = 0; i < segments.size() - 1; i++)
			draw_a_segment(segments[i], segments[i + 1], color_index, face_dis);
	}
}

void RemeshingMenu::draw_a_point(const Eigen::Vector3d v, const int color_index, const double face_dis) {
	Eigen::MatrixXd vec;
	vec.resize(1, 3);
	vec.row(0) = v;

	if (face_dis > 0.)
		vec.row(0) = v + face_vectors[CGAL_Closest_Face(igl_tree, v)].normal * face_dis;

    Eigen::RowVector3d color;
    switch (color_index) {
    case 0:
        color = { 0.8, 0.2, 0.2 }; break; // red
    case 1:
        color = { 0.2, 0.8, 0.2 }; break; // green
    case 2:
        color = { 0.2, 0.2, 0.8 }; break; // blue
    case 3:
        color = { 0.9, 0.5, 0.1 }; break; // orange
    case 4:
        color = { 0.0, 0.0, 0.0 }; break; // black
    }
    viewer->data().add_points(vec, color);
}

void RemeshingMenu::draw_points(const std::vector<Eigen::Vector3d> & vecs, const int color_index, const double face_dis) {
	for (int i = 0; i < vecs.size(); i++) draw_a_point(vecs[i], color_index, face_dis);
}

bool RemeshingMenu::highlight_symmetries() {
	int face_hit = -1;
	if (model_loaded()) {
		int fid;
		Eigen::Vector3f bc;
		double x = viewer->current_mouse_x;
		double y = viewer->core().viewport(3) - viewer->current_mouse_y;
		if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y),
			viewer->core().view, viewer->core().proj,
			viewer->core().viewport, V, F, fid, bc)) { face_hit = fid; }
    }
    if (face_hit >= 0) {
        std::vector<int> highlights = symmetrizer.symmetric_faces(face_hit);
        Eigen::MatrixXd colors;
        colors.resize(F.rows(), 3);
        colors.setConstant(1.0);
        for (auto f : highlights) {
            colors.row(f) = Eigen::Vector3d(1.0, 0.8, 0.0);
        }
        viewer->data().set_colors(colors);
        return true;
    }
    return false;
}

std::vector<int> RemeshingMenu::v_symmetry_axes() {
    std::vector<int> symmetries;
    if (symmetrizer.has_vertex_symmetry()) {
        if (symmetry_mode_yz && symmetrizer.has_vertex_symmetry(0)) {
            symmetries.push_back(0);
        }
        if (symmetry_mode_xz && symmetrizer.has_vertex_symmetry(1)) {
            symmetries.push_back(1);
        }
        if (symmetry_mode_xy && symmetrizer.has_vertex_symmetry(2)) {
            symmetries.push_back(2);
        }
    }
    return symmetries;
}

std::vector<int> RemeshingMenu::f_symmetry_axes() {
	std::vector<int> symmetries;
	if (symmetrizer.has_face_symmetry()) {
		if (symmetry_mode_yz && symmetrizer.has_face_symmetry(0)) {
			symmetries.push_back(0);
		}
		if (symmetry_mode_xz && symmetrizer.has_face_symmetry(1)) {
			symmetries.push_back(1);
		}
		if (symmetry_mode_xy && symmetrizer.has_face_symmetry(2)) {
			symmetries.push_back(2);
		}
	}
	return symmetries;
}

void RemeshingMenu::save_ctrlz() {
	if (temps.size() < 20) {
		auto get_edge_face_path = [](const std::string & filename, std::string & mesh_path_temp, std::string & face_path_temp, std::string & edge_path_temp) {
			std::size_t found = filename.find(".obj");
			if (found != std::string::npos) {
				face_path_temp = filename.substr(0, found) + "_temp.face";
				edge_path_temp = filename.substr(0, found) + "_temp.edge";
				mesh_path_temp = filename;
			} else {
				face_path_temp = filename + "_temp.face";
				edge_path_temp = filename + "_temp.edge";
				mesh_path_temp = filename + ".obj";
			}
		};

		std::string filename, mesh_path_temp, face_path_temp, edge_path_temp;
		filename = "ctrlz_" + std::to_string(temps.size()) + ".obj";
		get_edge_face_path(filename, mesh_path_temp, face_path_temp, edge_path_temp);

		TEMPDATA temp = { mesh_path_temp, face_path_temp, edge_path_temp };
		temps.emplace_back(temp);
		save(filename);

	} else {
		remove(temps[0].mesh.c_str());
		remove(temps[0].face.c_str());
		remove(temps[0].edge.c_str());
		for (int i = 1; i < temps.size(); i++) {
			rename(temps[i].mesh.c_str(), temps[i - 1].mesh.c_str());
			rename(temps[i].face.c_str(), temps[i - 1].face.c_str());
			rename(temps[i].edge.c_str(), temps[i - 1].edge.c_str());
		}
		save(temps.back().mesh);
	}
}

}
