#include "labeling_ui.h"

#include <imgui/imgui.h>
#include <igl/png/readPNG.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/file_dialog_save.h>
#include <igl/writeDMAT.h>
#include <igl/read_triangle_mesh.h>
#include <igl/readDMAT.h>
#include <igl/barycentric_to_global.h>
#include <igl/barycentric_coordinates.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#include "read_quad_mesh.h"

#include "glyph.h"
#include "glyphs.h"

#include "texture.h"

#include "vkmp/scheduler.hpp"

#include <fstream>

namespace ImGui
{
	static auto vector_getter = [](void* vec, int idx, const char** out_text)
	{
		auto& vector = *static_cast<std::vector<std::string>*>(vec);
		if (idx < 0 || idx >= static_cast<int>(vector.size())) { return false; }
		*out_text = vector.at(idx).c_str();
		return true;
	};

	bool Combo(const char* label, int* currIndex, std::vector<std::string>& values)
	{
		if (values.empty()) { return false; }
		return Combo(label, currIndex, vector_getter,
			static_cast<void*>(&values), values.size());
	}

	bool ListBox(const char* label, int* currIndex, std::vector<std::string>& values)
	{
		if (values.empty()) { return false; }
		return ListBox(label, currIndex, vector_getter,
			static_cast<void*>(&values), values.size());
	}

}

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
		if (auto_solve) {
			M.optimize_topology();
		}

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

		debug_index = viewer->append_mesh();

		// Set the data index back to the underlying mesh
		viewer->selected_data_index = base_index;

		mesh_loaded = true;
	}

	void LabelingUI::load_generator()
	{
		generator_path = igl::file_dialog_open();
		if (generator_path.size() > 0) {
			has_generator = true;
			int last_sep = generator_path.find_last_of('\\');
			std::string bc_file = generator_path.substr(0, last_sep) + "/bc.dmat";
			igl::readDMAT(bc_file, gen_coords);
		}
	}

	void LabelingUI::generate_variation(std::string args)
	{
		std::string command = generator_path + " " + args + " gen_temp.obj";
		system(command.c_str());
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;
		igl::read_triangle_mesh("gen_temp.obj", V, F);

		Eigen::MatrixXd new_coords = igl::barycentric_to_global(V, F, gen_coords);

		igl::writeOBJ("coarse_temp.obj", new_coords, M.F_q);

		load_variation("coarse_temp.obj");

		/*
		M.V.block(0, 0, new_coords.rows(), 3) = new_coords;
		
		M.side_lengths.clear();
		for (int q = 0; q < M.m; ++q) {
			for (int i = 0; i < 4; ++i) {
				M.V.row(M.n + q) = M.V.row(4 * q + 0) / 4 + M.V.row(4 * q + 1) / 4 + M.V.row(4 * q + 2) / 4 + M.V.row(4 * q + 3) / 4;
				double len = (M.V.row(M.F_q(q, (i + 1) % 4)) - M.V.row(M.F_q(q, i))).norm();
				M.side_lengths.push_back(len);	
			}
		}
		((LabeledQuadMesh)(M)).init();
		M.update_textures();

		viewer->selected_data_index = overlay_index;
		viewer->data().clear();

		viewer->data().set_mesh(M.V, M.F_t);
		
		viewer->data().set_mesh(M.LV, M.LF);
		viewer->data().set_texture(R, G, B, A);
		viewer->data().set_uv(M.UV);
		viewer->data().show_texture = true;
		viewer->data().show_lines = false;
		viewer->data().set_colors(M.C);
		*/
	}

	void LabelingUI::load_variation(std::string filename)
	{
		LabeledQuadMesh variation;
		
		read_quad_mesh(filename, variation, planarize);
		M.V = variation.V;
		M.LV = variation.LV;
		M.side_lengths.clear();
		for (int q = 0; q < M.m; ++q) {
			for (int i = 0; i < 4; ++i) {
				double len = (M.V.row(M.F_q(q, (i + 1) % 4)) - M.V.row(M.F_q(q, i))).norm();
				M.side_lengths.push_back(len);
			}
		}
		M.update_textures();

		viewer->selected_data_index = overlay_index;
		viewer->data().clear();
		viewer->data().set_mesh(M.LV, M.LF);
		viewer->data().set_texture(R, G, B, A);
		viewer->data().set_uv(M.UV);
		viewer->data().show_texture = true;
		viewer->data().show_lines = false;
		viewer->data().set_colors(M.C);

	}


	bool LabelingUI::mouse_down(int button, int modifier) {
		if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_down(button, modifier)) return true;
		
		if (modifier & IGL_MOD_CONTROL) {
			// Get the starting side
			
			if (current_tool == ORIENTER) {
				if (button == (int)igl::opengl::glfw::Viewer::MouseButton::Left) {
					orienter_mode = LOOP;
				}
				else {
					orienter_mode = YARN;
				}
			}

			int fid;
			Eigen::Vector3f bc;
			if (pick_face(fid, bc)) {

				drag_start_side = fid;
				last_drag_side = fid;

				is_dragging = true;
				dragging_button = button;

				

				return true;
			}
			else {
				drag_start_side = -1;
				last_drag_side = -1;

				is_dragging = true;
				dragging_button = button;
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
				solve_topology();
			}
			update_mesh();
			return true;
		}
		else {

			if (current_tool == MEASURER) {
				if (measurer_mode == SHAPE_MODE) {
					auto dx = (viewer->down_mouse_x - viewer->current_mouse_x);
					auto dy = (viewer->down_mouse_y - viewer->current_mouse_y);
					auto dist2 = dx * dx + dy * dy;
					if (dist2 > 5) return false;
					int fid;
					Eigen::Vector3f bc;
					if (pick_face(fid, bc)) {
						M.set_shaping(fid, shaping_brush, short_row_brush);
						update_mesh();
					}
				}
				else {
					int fid;
					Eigen::Vector3f bc;
					if (pick_face(fid, bc)) {
						int side = outgoing_side(fid, bc);
						if (side >= 0) {
							auto loop = M.side_loop(side);
							last_size_line = M.add_size_line(loop, use_last_sizing ? last_size_line : -1);
						}
					}
				}

			}

			if (current_tool == TEXTURER) {
				auto dx = (viewer->down_mouse_x - viewer->current_mouse_x);
				auto dy = (viewer->down_mouse_y - viewer->current_mouse_y);
				auto dist2 = dx * dx + dy * dy;
				if (dist2 > 5) return false;
				int fid;
				Eigen::Vector3f bc;
				if (pick_face(fid, bc)) {
					M.set_texture(fid, current_texture);
					update_mesh();
				}

			}

			if (current_tool == SEAMER) {

				// Make sure we aren't actually just moving the camera

				auto dx = (viewer->down_mouse_x - viewer->current_mouse_x);
				auto dy = (viewer->down_mouse_y - viewer->current_mouse_y);
				auto dist2 = dx * dx + dy * dy;
				if (dist2 > 5) return false;

				int fid;
				Eigen::Vector3f bc;
				if (pick_face(fid, bc)) {
					int side = fid;
					int edge = M.sides_to_edges[side];
					int closest_vertex;
					bc.maxCoeff(&closest_vertex);
					int vtx = M.F_t(fid, closest_vertex);

					if (button == (int)igl::opengl::glfw::Viewer::MouseButton::Left) {

						// If we have selected an existing seam option toggle it
						if (edge >= 0 && M.edges[edge].seam >= 0) {
							M.toggle_seam(side);
							if (auto_solve) {
								solve_topology();
							}
							update_mesh();
						}
						else { // Otherwise extend a seam from the closest vertex
							int side = outgoing_side(fid, bc);
							if (side >= 0) {
								auto edges = M.trace_seam(side);
								M.update_textures();
								std::vector<int> new_seam_sides;
								for (auto e : edges) {
									if (e >= 0) {
										new_seam_sides.push_back(M.edges_to_sides(e, 0));
									}
								}
								M.add_seam(new_seam_sides);
								update_mesh();
							}
						}
						return true;
					}
					else if (button == (int)igl::opengl::glfw::Viewer::MouseButton::Right) {
						M.toggle_special_vertex(vtx);
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

				if (current_tool == TEXTURER) {
					if (M.flip_side(fid) == last_drag_side) {
						M.set_texture(fid, current_texture);
						M.set_texture(M.flip_side(fid), current_texture);
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

				if (current_tool == MEASURER && measurer_mode == SHAPE_MODE) {
					if(M.flip_side(fid) == last_drag_side) {
						M.set_shaping(fid, shaping_brush, short_row_brush);
						M.set_shaping(M.flip_side(fid), shaping_brush, short_row_brush);
						update_mesh();
					}
					last_drag_side = fid;
					return true;
				}
			}
			else {
				if (current_tool == ORIENTER) {
					if (last_drag_side >= 0) {
						M.paint_direction(last_drag_side, -1, orienter_mode);
						update_mesh();
						last_drag_side = -1;
						return true;
					}
				}
			}
		}

		if (current_tool == SEAMER) {

			int fid;
			Eigen::Vector3f bc;
			M.update_textures();
			if (pick_face(fid, bc)) {
				int edge = M.sides_to_edges[fid];
				int closest_vertex;
				bc.maxCoeff(&closest_vertex);
				int vtx = M.F_t(fid, closest_vertex);

				// If we are hovering over a potential seam, highlight it
				if (edge >= 0 && M.edges[edge].seam >= 0) {
					int seam_id = M.edges[edge].seam;
					for (int e : M.seam_edges[seam_id]) {
						M.set_glyph(M.edge_slots[e], glyphs::SOLID_LINE, color::ORANGE);
					}
				}
				else { // Otherwise, highlight a potential seam extending from the nearest vertex
					M.update_textures();
					int side = outgoing_side(fid, bc);
					if (side >= 0) {
						auto edges = M.trace_seam(side);
						for (auto e : edges) {
							if (e >= 0) {
								M.set_glyph(M.edge_slots[e], glyphs::SEAM, color::RED);
							}
						}
					}
				}
				// Highlight any potential special vertices
				M.set_glyph(M.vertex_slots[vtx], glyphs::CIRCLE, color::ORANGE);
			}
			update_mesh();
		}

		if (current_tool == MEASURER && measurer_mode == SIZE_MODE) {
			int fid;
			Eigen::Vector3f bc;
			M.update_textures();
			if (pick_face(fid, bc)) {
				int side = outgoing_side(fid, bc);
				if (side >= 0) {
					auto loop = M.side_loop(side);
					for (int i : loop) {
						M.set_glyph(M.half_edge_slots[i], glyphs::SOLID_LINE, color::RED);
					}
				}
			}
			update_mesh();
		}

		return false;
	}


	void LabelingUI::draw_viewer_menu() {
		load_textures();
		load_knitting_textures();
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
				if (mesh_loaded) {
					M.update_textures();
				}
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

		if (ImGui::Button("Export Coarse Knit Mesh")) {
			auto filename = igl::file_dialog_save();
			if (filename.size() > 0) {
				M.export_data(filename);
			}
		}

		if (ImGui::Button("Load Stitches")) {
			auto filename = igl::file_dialog_open();
			if (filename.size() > 0) {
				KnitGraph kg;
				kg.load_st(filename);
				auto vis = kg.visualize_stitches(kg.stitches);
				vis.display(viewer->data());
			}
		}

		if (ImGui::Button("Load Variation")) {
			std::string filename = igl::file_dialog_open();
			load_variation(filename);
		}

		if (ImGui::Button("Load Generator")) {
			load_generator();
		}

		if (has_generator) {
			static char args_buff[128] = "1 1 1 1 1";
			ImGui::InputText("Generator args", args_buff, IM_ARRAYSIZE(args_buff));

			if (ImGui::Button("Generate Variation")) {
				std::string args(args_buff);
				generate_variation(args);
			}
		}


		if (ImGui::Button("Schedule Stitches File")) {
			auto filename = igl::file_dialog_open();
			if (filename.size() > 0) {
				KnitGraph kg;
				kg.load_st(filename);
				auto vis = kg.visualize_stitches(kg.stitches);
				vis.display(viewer->data());
				auto filename2 = igl::file_dialog_save();
				if (filename2.size() > 0) {

					vkmp::Scheduler s;
					s.stitches = kg.stitches;

					std::map<int, std::pair<int, int>> yarn_mappings;
					yarn_mappings[0] = std::make_pair(1, -1);
					yarn_mappings[1] = std::make_pair(2, -1);
					s.do_schedule(yarn_mappings, true, -1);
					s.write_schedule(filename + ".js");
					std::string scripts_dir = SCRIPTS_DIR;
					std::string node_path = "NODE_PATH=" + scripts_dir + "\\";
					putenv(node_path.c_str());
					std::string command_1 = "node " + filename + ".js";
					std::cout << "Trying to run:\n" << command_1 << std::endl;
					system(command_1.c_str());
					std::string command_2 = "node " + scripts_dir + "\\knitout-to-dat.js " + filename + ".k " + filename + ".dat";
					std::cout << "Trying to run:\n" << command_2 << std::endl;
					system(command_2.c_str());
					kg.generate_instructions(filename2);
				}
			}
		}

		if (ImGui::Checkbox("Auto-Solve", &auto_solve)) {
			if (auto_solve) {
				solve_topology();
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
			
			// Leaving this here because it's useful for figuring out ImGui Stuff...
			//ImGui::ShowStyleEditor();

			ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(
				(float)textures[current_texture].color(0),
				(float)textures[current_texture].color(1),
				(float)textures[current_texture].color(2),
				(float)textures[current_texture].color(3)));
			ImGui::Combo("", &current_texture, texture_names);
			ImGui::PopStyleColor();
			ImGui::PopItemWidth();

			if (ImGui::Button("Fill")) {
				for (auto& q : M.quads) {
					q.texture_id = current_texture;
				}
				M.update_textures();
			}

		}
		mode_selector(SEAMER, seamer_pressed, seamer_tex, seamer_instructions, "Seaming Tool");
		if (current_tool == SEAMER) {
			if (ImGui::Button("Fix All")) {
				if (mesh_loaded) {
					for (auto& seam : M.seams) {
						seam->is_fixed = true;
					}
					M.update_textures();
					update_mesh();
				}
			}
			if (ImGui::Button("UnFix All")) {
				if (mesh_loaded) {
					for (auto& seam : M.seams) {
						seam->is_fixed = false;
					}
					M.update_textures();
					update_mesh();
				}
			}
		}
		mode_selector(ORIENTER, orienter_pressed, orienter_tex, orienter_instructions, "Orienting Tool");
		if (current_tool == ORIENTER) {
			ImGui::RadioButton("Loop", (int*)& orienter_mode, LOOP);
			ImGui::RadioButton("Yarn", (int*)& orienter_mode, YARN);
		}
		mode_selector(MEASURER, measurer_pressed, measurer_tex, measurer_instructions, "Constraints Tool");
		if (current_tool == MEASURER) {

			ImGui::RadioButton("Shaping", (int*)& measurer_mode, SHAPE_MODE);
			ImGui::RadioButton("Sizing", (int*)& measurer_mode, SIZE_MODE);

			if (measurer_mode == SHAPE_MODE) {
				ImGui::Text("Inc/Dec");
				ImGui::RadioButton("None", (int*)& shaping_brush, NONE);
				ImGui::RadioButton("Left (In)", (int*)& shaping_brush, IN_SIDE);
				ImGui::RadioButton("Right (Out)", (int*)& shaping_brush, OUT_SIDE);
				ImGui::RadioButton("Both Sides", (int*)& shaping_brush, BOTH_SIDES);
				ImGui::RadioButton("Distributed", (int*)& shaping_brush, DISTRIBUTED);
				ImGui::Text("Short Rows");
				ImGui::RadioButton("None", (int*)& short_row_brush, NONE);
				ImGui::RadioButton("Top (Out)", (int*)& short_row_brush, OUT_SIDE);
				ImGui::RadioButton("Bottom (In)", (int*)& short_row_brush, IN_SIDE);

				if (ImGui::Button("Apply Everywhere")) {
					for (int i = 0; i < M.quads.size(); ++i) {
						M.set_shaping(4 * i, shaping_brush, short_row_brush);
					}
				}

			}
			else {
				ImGui::Checkbox("Use Last Sizing", &use_last_sizing);
				ImGui::Text("Click a line to make it a critical line.");
			}
		}
		ImGui::Text(instructions.c_str());
		
		if (ImGui::Button("Optimize Geometry")) {
			optimize_geometry();
		}

		if (ImGui::Button("Save Coarse Graph")) {
			save_graph(viewer->data(graph_index));
		}

		if (ImGui::Button("Generate Knitting Instructions")) {
			generate_instructions();
		}

		ImGui::InputInt("Depth", &depth);
		ImGui::Checkbox("Use Stacked Planner", &use_stacked_planner);

		if (ImGui::Button("Generate Knit Graph")) {
			extract_fine_graph();
		}
		if (ImGui::Button("Save Knit Graph")) {
			save_graph(viewer->data(knit_graph_index));
		}

		if (ImGui::Button("Trace Graph")) {
			trace_graph();
		}
		if (ImGui::Button("Save Traced Graph")) {
			save_graph(viewer->data(traced_graph_index));
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

		if (ImGui::Checkbox("Show Debug", &show_debug)) {
			set_layer(debug_index, show_debug);
		}

		if (ImGui::DragInt("Minimizer Timeout", &minimizer_timeout, 1.0, 1, 60)) {
			M.minimizer_timeout = (unsigned int)minimizer_timeout;
		}

		if (ImGui::DragInt("Side Tolerance", &M.edge_tolerance, 1.0, 0, 10)) {
			M.geometry_solved = false;
		}

		if (ImGui::DragInt("Course Tolerance", &M.course_tolerance, 1.0, 0, 10)) {
			M.geometry_solved = false;
		}

		if (ImGui::DragInt("Wale Tolerance", &M.wale_tolerance, 1.0, 0, 10)) {
			M.geometry_solved = false;
		}

		if (ImGui::DragInt("Critical Tolerance", &M.critical_tolerance, 1.0, 0, 10)) {
			M.geometry_solved = false;
		}

		if (ImGui::InputDouble("Tolerance", &M.tolerance)) {
			M.geometry_solved = false;
		}

		if (ImGui::InputDouble("Scale", &M.scale)) {
			M.geometry_solved = false;
		}

		if (ImGui::InputDouble("Stitch Gauge", &M.stitch_gauge)) {
			M.geometry_solved = false;
		}

		if (ImGui::InputDouble("Row Gauge", &M.row_gauge)) {
			M.geometry_solved = false;
		}
		if (ImGui::Checkbox("Symmetrize", &M.symmetrize)) {
			M.geometry_solved = false;
		}

		if (ImGui::Checkbox("White BG", &white_bg)) {
			if (white_bg) {
				viewer->core(0).background_color = Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
			}
			else {
				viewer->core(0).background_color = Eigen::Vector4f(8.0, 8.0, 8.0, 1.0);

			}
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
			//viewer->data(knit_graph_index).add_label(V.row(i), std::to_string(i));
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

			viewer->data(traced_graph_index).clear_labels();
		}
	}

	void LabelingUI::generate_instructions()
	{
		//optimize_geometry();
		auto CKG = M.get_dual();
		auto KG = CKG.build_graph();
		KG.contract();
		KG.generate_instructions(igl::file_dialog_save(), depth, !use_stacked_planner);
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

	void LabelingUI::solve_topology()
	{
		bool sat = M.optimize_topology();

		if (!sat) {
			std::cout << "UNSAT!" << std::endl;
		}

		update_mesh();
		// TODO - Handle invalid constraints
	}

	void LabelingUI::save_graph(const igl::opengl::ViewerData& data)
	{
		std::string filebase = igl::file_dialog_save();
		if (filebase.size() > 0) {
			auto point_file = filebase + "_points.dmat";
			auto line_file = filebase + "_lines.dmat";
			igl::writeDMAT(point_file, data.points, true);
			igl::writeDMAT(line_file, data.lines, true);
			data.lines;
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

	int LabelingUI::outgoing_side(int fid, const Eigen::Vector3f& bc)
	{
		if (!mesh_loaded) return -1;

		int opp_side = M.flip_side(fid);

		return bc(0) > bc(1) ? fid : (opp_side >= 0 ? opp_side : fid);
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
	void LabelingUI::load_knitting_textures()
	{
		if (!knitting_textures_loaded) {
			init_textures();
			for (auto texture : textures) {
				texture_names.push_back(texture.name);
			}

			knitting_textures_loaded = true;
		}
	}
}