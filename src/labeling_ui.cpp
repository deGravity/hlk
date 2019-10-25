#include "labeling_ui.h"

#include <imgui/imgui.h>

namespace hlk {

	bool LabelingUI::mouse_down(int button, int modifier) {
		if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_down(button, modifier)) return true;
		
		return false;
	}

	bool LabelingUI::mouse_up(int button, int modifier) {
		if (is_dragging) {
			// Finalize Dragging Action

			is_dragging = false;

			if (auto_solve) {
				// TODO - Call Solver if necessary
			}

			return true;
		}



		return false;
	}

	bool LabelingUI::mouse_move(int mouse_x, int mouse_y) {
		if (igl::opengl::glfw::imgui::ImGuiMenu::mouse_move(mouse_x, mouse_y)) return true;

		return false;
	}


	void LabelingUI::draw_viewer_menu() {
		load_textures();
		if (ImGui::ImageButton((void*)(intptr_t)(current_tool == ERASER ? eraser_pressed : eraser_tex), ImVec2(32, 32))) {
			current_tool = ERASER;
			instructions = eraser_instructions;
		}
		if (current_tool == ERASER) {
			ImGui::RadioButton("Erase Orientations", (int*)&eraser_mode, ERASE_ORIENTATIONS);
			ImGui::RadioButton("Erase Seams", (int*)&eraser_mode, ERASE_SEAMS);
			ImGui::RadioButton("Erase Textures", (int*)&eraser_mode, ERASE_TEXTURES);
			ImGui::RadioButton("Erase Constraints", (int*)&eraser_mode, ERASE_CONSTRAINTS);
		}
		if (ImGui::ImageButton((void*)(intptr_t)(current_tool == TEXTURER ? brush_pressed : brush_tex), ImVec2(32, 32))) {
			current_tool = TEXTURER;
			instructions = texturer_instructions;
		}
		if (current_tool == TEXTURER) {

		}
		if (ImGui::ImageButton((void*)(intptr_t)(current_tool == SEAMER ? seamer_pressed : seamer_tex), ImVec2(32, 32))) {
			current_tool = SEAMER;
			instructions = seamer_instructions;
		}
		if (current_tool == SEAMER) {

		}
		if (ImGui::ImageButton((void*)(intptr_t)(current_tool == ORIENTER ? orienter_pressed : orienter_tex), ImVec2(32, 32))) {
			current_tool = ORIENTER;
			instructions = orienter_instructions;
		}
		if (current_tool == ORIENTER) {

		}
		if (ImGui::ImageButton((void*)(intptr_t)(current_tool == MEASURER ? measurer_pressed : measurer_tex), ImVec2(32, 32))) {
			current_tool = MEASURER;
			instructions = measurer_instructions;
		}
		if (current_tool == MEASURER) {

		}
		ImGui::Text(instructions.c_str());
	}

	// Cannot call this until _after_ a viewer window is open
	void LabelingUI::load_textures() {
		if (!textures_loaded) {
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