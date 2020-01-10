#include "labeling_ui.h"

#include <imgui/imgui.h>
#include <igl/png/readPNG.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/file_dialog_save.h>

#include "read_quad_mesh.h"

#include "glyph.h"
#include "glyphs.h"

#include <fstream>

namespace hlk {
	bool LabelingUI::load_quad_mesh_file()
	{
		std::string filename = igl::file_dialog_open();
		if (filename.size() > 0) {
			load_quad_mesh_file(filename);
			return true;
		}
		return false;
	}

	void LabelingUI::load_quad_mesh_file(std::string filename)
	{
		read_quad_mesh(filename, M, planarize);

		init_quad_mesh_display();
	}

	void LabelingUI::init_quad_mesh_display()
	{
		// Setup the base mesh
		viewer->data().clear();
		viewer->data().set_mesh(M.V, M.F_t);
		viewer->data().set_colors(Eigen::RowVector4d(1.0, 1.0, 1.0, 1.0));
		viewer->data().show_faces = false;
		viewer->data().show_lines = false;
		viewer->data().show_texture = false;

		base_index = viewer->selected_data_index;

		if (!mesh_loaded) {
			overlay_index = viewer->append_mesh();
		}
		else {
			viewer->selected_data_index = overlay_index;
			viewer->data().clear();
		}
		viewer->data().set_mesh(M.LV, M.LF);
		viewer->data().set_texture(R, G, B, A);
		viewer->data().set_uv(M.UV);
		viewer->data().show_texture = true;
		viewer->data().show_lines = false;
		viewer->data().set_colors(M.C);

		// Add a mesh-data object for a graph overlay. Don't use it yet
		graph_index = viewer->append_mesh();
		viewer->data().show_faces = false;
		viewer->data().show_texture = false;

		knit_graph_index = viewer->append_mesh();
		viewer->data().show_faces = false;
		viewer->data().show_texture = false;

		traced_graph_index = viewer->append_mesh();
		viewer->data().show_faces = false;
		viewer->data().show_texture = false;


		// Set the data index back to the underlying mesh
		viewer->selected_data_index = base_index;

		mesh_loaded = true;
	}


	bool LabelingUI::mouse_down(int button, int modifier) {
		if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_down(button, modifier)) return true;
		
		if (modifier & IGL_MOD_CONTROL) {
			// Get the starting side
			
			int fid;
			Eigen::Vector3f bc;
			if (pick_face(fid, bc)) {

				drag_start_side = fid;
				last_drag_side = fid;

				is_dragging = true;
				dragging_button = button;

				if (current_tool == ORIENTER) {
					std::cout << "mouse button = " << button << std::endl;
					if (button == (int)igl::opengl::glfw::Viewer::MouseButton::Left) {
						orienter_mode = LOOP;
					}
					else {
						orienter_mode = YARN;
					}
				}

				return true;
			}

			return false;
		}

		return false;
	}

	bool LabelingUI::mouse_up(int button, int modifier) {
		if (is_dragging) {
			// Finalize Dragging Action

			is_dragging = false;
			drag_start_side = -1;

			if (auto_solve) {
				bool sat = M.optimize_topology();

				if (!sat) {
					std::cout << "UNSAT!" << std::endl;
				}
				// TODO - Handle invalid constraints
			}
			update_mesh();
			return true;
		}
		else {

			if (current_tool == SEAMER) {
				int fid;
				Eigen::Vector3f bc;
				if (pick_face(fid, bc)) {
					int side = fid;
					int edge = M.sides_to_edges[side];
					if (edge >= 0 && M.edges[edge].seam >= 0) {
						M.toggle_seam(side);
						if (auto_solve) {
							bool sat = M.optimize_topology();
							update_mesh();
							if (!sat) {
								std::cout << "UNSAT!" << std::endl;
							}
							// TODO - Handle invalid constraints
						}
						update_mesh();	
					}
					else {

						int closest_vertex;
						bc.maxCoeff(&closest_vertex);
						// Find the outgoing side from the closest vertex,
						// or -1 if non-quad vertex or boundary
						int side = -1;
						if (closest_vertex == 0) {
							side = fid;
						}
						if (closest_vertex == 1) {
							side = M.flip_side(fid);
						}
						if (side >= 0) {
							//if (M.vertex_in_seam[M.F_t(fid, closest_vertex)]) {
								//side = M.flip_side(fid);
							//}
							if (side >= 0) {
								M.update_textures();
								auto loop = M.side_loop(side);
								int end = 0;
								std::vector<int> new_seam_sides;
								for (end = 0; end < loop.size(); ++end) {
									new_seam_sides.push_back(end);
									if (M.vertex_in_seam[M.side_v(loop[end])]) {
										break;
									}
								}
								new_seam_sides = loop;
								M.add_seam(new_seam_sides);
								update_mesh();
							}
						}
					}
				}
			}

		}

		return false;
	}

	bool LabelingUI::mouse_move(int mouse_x, int mouse_y) {
		if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_move(mouse_x, mouse_y)) return true;

		if (is_dragging) {
			int fid;
			Eigen::Vector3f bc;
			if (pick_face(fid, bc)) {

				if (current_tool == ORIENTER) {

					if (M.flip_side(fid) == last_drag_side) {
						M.paint_direction(last_drag_side, fid, orienter_mode);
						update_mesh();
					}
					last_drag_side = fid;
					return true;
				}

				if (current_tool == ERASER) {
					if (eraser_mode == ERASE_ORIENTATIONS) {
						M.erase_orientation(fid);
						update_mesh();
					}
				}
			}
		}

		if (current_tool == SEAMER) {
			int fid;
			Eigen::Vector3f bc;
			if (pick_face(fid, bc)) {

				int edge = M.sides_to_edges[fid];
				if (edge >= 0 && M.edges[edge].seam >= 0) {
					M.update_textures();
					int seam_id = M.edges[edge].seam;
					for (int e : M.seam_edges[seam_id]) {
						M.set_glyph(M.edge_slots[e], glyphs::SOLID_LINE, color::ORANGE);
					}
					update_mesh();
				}
				else {

					int closest_vertex;
					bc.maxCoeff(&closest_vertex);
					// Find the outgoing side from the closest vertex,
					// or -1 if non-quad vertex or boundary
					int side = -1;
					if (closest_vertex == 0) {
						side = fid;
					}
					if (closest_vertex == 1) {
						side = M.flip_side(fid);
					}
					if (side >= 0) {
						if (M.vertex_in_seam[M.F_t(fid, closest_vertex)]) {
							side = M.flip_side(fid);
						}
						if (side >= 0) {
							M.update_textures();
							auto loop = M.side_loop(side);
							int end = 0;
							for (end = 0; end < loop.size(); ++end) {
								if (M.vertex_in_seam[M.side_v(loop[end])]) {
									break;
								}
							}
							for (int i = 0; i < loop.size(); ++i) {
								int s = loop[i];
								int e = M.sides_to_edges[s];
								if (e >= 0) {
									M.set_glyph(M.edge_slots[e], glyphs::SEAM, color::RED);
								}
							}
							update_mesh();
						}
					}
				}
			}
		}

		return false;
	}


	void LabelingUI::draw_viewer_menu() {
		load_textures();
		auto tooltip = [](std::string text) {
			if (ImGui::IsItemHovered()) {
				ImGui::BeginTooltip();
				ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
				ImGui::TextUnformatted(text.c_str());
				ImGui::PopTextWrapPos();
				ImGui::EndTooltip();
			}
		};
		auto mode_selector = [&](Tool t, GLuint pressed, GLuint unpressed, std::string tool_instructions, std::string name) {
			if (ImGui::ImageButton((void*)(intptr_t)(current_tool == t ? pressed : unpressed), ImVec2(32, 32))) {
				current_tool = t;
				instructions = tool_instructions;
			}
			tooltip(name);
		};


		if (ImGui::Button("Load Quad Mesh")) {
			load_quad_mesh_file();
		}

		if (ImGui::Button("Load Coarse Knit Mesh")) {
			auto filename = igl::file_dialog_open();
			if (filename.size() > 0) {
				load_coarse_knit_mesh(filename);
			}
		}

		if (ImGui::Button("Save Coarse Knit Mesh")) {
			auto filename = igl::file_dialog_save();
			if (filename.size() > 0) {
				save_coarse_knit_mesh(filename);
			}
		}

		mode_selector(ERASER, eraser_pressed, eraser_tex, eraser_instructions, "Eraser Tool");
		if (current_tool == ERASER) {
			ImGui::RadioButton("Erase Orientations", (int*)&eraser_mode, ERASE_ORIENTATIONS);
			ImGui::RadioButton("Erase Seams", (int*)&eraser_mode, ERASE_SEAMS);
			ImGui::RadioButton("Erase Textures", (int*)&eraser_mode, ERASE_TEXTURES);
			ImGui::RadioButton("Erase Constraints", (int*)&eraser_mode, ERASE_CONSTRAINTS);
		}
		mode_selector(TEXTURER, brush_pressed, brush_tex, texturer_instructions, "Texturing Tool");
		if (current_tool == TEXTURER) {
			ImGui::SameLine();
			ImGui::PushItemWidth(100);
			ImGui::Combo("", &current_texture, textures.data(), textures.size());
			ImGui::PopItemWidth();
		}
		mode_selector(SEAMER, seamer_pressed, seamer_tex, seamer_instructions, "Seaming Tool");
		if (current_tool == SEAMER) {

		}
		mode_selector(ORIENTER, orienter_pressed, orienter_tex, orienter_instructions, "Orienting Tool");
		if (current_tool == ORIENTER) {
			ImGui::RadioButton("Loop", (int*)& orienter_mode, LOOP);
			ImGui::RadioButton("Yarn", (int*)& orienter_mode, YARN);
		}
		mode_selector(MEASURER, measurer_pressed, measurer_tex, measurer_instructions, "Constraints Tool");
		if (current_tool == MEASURER) {

		}
		ImGui::Text(instructions.c_str());
		
		if (ImGui::Button("Optimize Geometry")) {
			optimize_geometry();
		}

		if (ImGui::Button("Generate Knitting Instructions")) {
			generate_instructions();
		}

		if (ImGui::Button("Generate Knit Graph")) {
			extract_fine_graph();
		}

		if (ImGui::Button("Trace Graph")) {
			trace_graph();
		}

		if (ImGui::Checkbox("Show Mesh", &show_mesh)) {
			set_layer(overlay_index, show_mesh);
		}

		if (ImGui::Checkbox("Show Graph", &show_graph)) {
			set_layer(graph_index, show_graph);
		}

		if (ImGui::Checkbox("Show Knit Graph", &show_knit_graph)) {
			set_layer(knit_graph_index, show_knit_graph);
		}

		if (ImGui::Checkbox("Show Traced Graph", &show_traced_graph)) {
			set_layer(traced_graph_index, show_traced_graph);
		}

		if (ImGui::DragInt("Minimizer Timeout", &minimizer_timeout, 1.0, 1, 60)) {
			M.minimizer_timeout = (unsigned int)minimizer_timeout;
		}

		if (ImGui::InputDouble("Tolerance", &M.tollerance)) {
			M.geometry_solved = false;
		}

		if (ImGui::InputDouble("Scale", &M.scale)) {
			M.geometry_solved = false;
		}
	
	}

	void LabelingUI::update_mesh()
	{
		if (mesh_loaded) {
			viewer->data_list[overlay_index].set_uv(M.UV);
			viewer->data_list[overlay_index].set_colors(M.C);
		}
	}

	void LabelingUI::optimize_geometry()
	{
		M.optimize_geometry();
		extract_coarse_graph();
	}

	void LabelingUI::extract_coarse_graph()
	{
		Eigen::MatrixXd P, P_c, V, E_c, L_p;
		Eigen::MatrixXi E;
		std::vector<std::string> L;

		auto CKG = M.get_dual();
		CKG.visualize(P, P_c, V, E, E_c, L, L_p);

		viewer->data(graph_index).set_points(P, P_c);
		viewer->data(graph_index).set_edges(V, E, E_c);
		viewer->data(graph_index).labels_positions = L_p;
		viewer->data(graph_index).labels_strings = L;
		
	}

	void LabelingUI::extract_fine_graph()
	{
		auto CKG = M.get_dual();
		auto G = CKG.build_graph();
		G.contract();

		Eigen::MatrixXd V;
		Eigen::MatrixXi E;
		Eigen::MatrixXd C;

		G.build_mesh(0.1, 4, V, E, C);
		viewer->data(knit_graph_index).set_points(V, Eigen::RowVector3d(1.0, 1.0, 1.0));
		for (int i = 0; i < V.rows(); ++i) {
			viewer->data(knit_graph_index).add_label(V.row(i), std::to_string(i));
		}

		viewer->data(knit_graph_index).set_edges(V, E, C);
		viewer->data(knit_graph_index).line_width = 2.0f;
	}

	void LabelingUI::trace_graph()
	{
		auto CKG = M.get_dual();
		auto KG = CKG.build_graph();
		KG.contract();
		KG.trace();
		if (KG.traced) {
			auto vis = KnitGraph::visualize_stitches(KG.stitches);
			vis.display(viewer->data(traced_graph_index));
			viewer->data(traced_graph_index).line_width = 2.5;
			viewer->data(traced_graph_index).point_size = 10;
		}
	}

	void LabelingUI::generate_instructions()
	{
		optimize_geometry();
		auto CKG = M.get_dual();
		auto KG = CKG.build_graph();
		KG.contract();
		KG.generate_instructions(igl::file_dialog_save());
	}

	void LabelingUI::save_coarse_knit_mesh(std::string filename)
	{
		std::ofstream f(filename.c_str());
		M.save(f);
	}

	void LabelingUI::load_coarse_knit_mesh(std::string filename)
	{
		std::ifstream f(filename.c_str());
		M.load(f);
		M.update_textures();
		init_quad_mesh_display();
		extract_coarse_graph();
	}

	void LabelingUI::set_layer(int layer, bool on)
	{
		viewer->data(layer).show_faces = on;
		viewer->data(layer).show_lines = on;
		viewer->data(layer).show_overlay = on;
		viewer->data(layer).show_texture = on;
		if (on) {
			viewer->data(layer).label_color(3) = 1.0;
		}
		else {
			viewer->data(layer).label_color(3) = 0.0; 
		}
	}

	bool LabelingUI::pick_face(int& fid, Eigen::Vector3f& bc)
	{
		if (mesh_loaded) {
			double x = viewer->current_mouse_x;
			double y = viewer->core().viewport(3) - viewer->current_mouse_y;
			return igl::unproject_onto_mesh(
				Eigen::Vector2f(x, y),
				viewer->core().view,
				viewer->core().proj,
				viewer->core().viewport,
				viewer->data_list[base_index].V,
				viewer->data_list[base_index].F,
				fid,
				bc);
		}
		return false;
	}

	// Cannot call this until _after_ a viewer window is open
	void LabelingUI::load_textures() {
		if (!textures_loaded) {
			// TODO - This might come back to bite us later if we
			// want to set textures before the viewer loads.
			// Consider moving the glyphs loading to init or
			// constructor
			igl::png::readPNG("glyphs.png", R, G, B, A);
			igl::png::texture_from_file("eraser.png", eraser_tex);
			igl::png::texture_from_file("brush.png", brush_tex);
			igl::png::texture_from_file("seamer.png", seamer_tex);
			igl::png::texture_from_file("orienter.png", orienter_tex);
			igl::png::texture_from_file("measurer.png", measurer_tex);

			igl::png::texture_from_file("eraser_pressed.png", eraser_pressed);
			igl::png::texture_from_file("brush_pressed.png", brush_pressed);
			igl::png::texture_from_file("seamer_pressed.png", seamer_pressed);
			igl::png::texture_from_file("orienter_pressed.png", orienter_pressed);
			igl::png::texture_from_file("measurer_pressed.png", measurer_pressed);
			textures_loaded = true;
		}
	}
}