#pragma once

#include <igl/png/texture_from_file.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui/imgui.h>

namespace hlk {
	class LabelingUI : public igl::opengl::glfw::imgui::ImGuiMenu {
	public:

		void init(igl::opengl::glfw::Viewer* _viewer) {
			igl::opengl::glfw::imgui::ImGuiMenu::init(_viewer);
		}

		void draw_viewer_menu() {
			load_textures();
			if (ImGui::ImageButton((void*)(intptr_t)eraser_tex, ImVec2(32, 32))) {
				current_tool = ERASER;
			}
			if (ImGui::ImageButton((void*)(intptr_t)brush_tex, ImVec2(32, 32))) {
				current_tool = TEXTURER;
			}
			if (ImGui::ImageButton((void*)(intptr_t)seamer_tex, ImVec2(32, 32))) {
				current_tool = SEAMER;
			}
			if (ImGui::ImageButton((void*)(intptr_t)orienter_tex, ImVec2(32, 32))) {
				current_tool = ORIENTER;
			}
			if (ImGui::ImageButton((void*)(intptr_t)measurer_tex, ImVec2(32, 32))) {
				current_tool = MEASURER;
			}
		}

	private:

		enum Tool {
			ERASER, // Remove constraints
			TEXTURER, // Paint on textures
			SEAMER, // Add, remove, and split seams
			ORIENTER, // Paint directions and orientations
			MEASURER // Tape Measure - set lines of constrained length
		};

		enum EraserMode {
			ERASE_ORIENTATIONS,
			ERASE_SEAMS,
			ERASE_TEXTURES,
			ERASE_CONSTRAINTS
		};

		enum OrientationMode {
			COURSE,
			WALE
		};

		// Modes and settings

		Tool current_tool;
		EraserMode eraser_mode;
		OrientationMode orientation_direction;

		// Whether to re-run the solver on mouse-up or not
		bool auto_solve;
		

		// Texture Handles for UI
		bool textures_loaded = false;
		GLuint eraser_tex, brush_tex, seamer_tex, orienter_tex, measurer_tex;

		void load_textures() {
			if (!textures_loaded) {
				igl::png::texture_from_file("eraser.png", eraser_tex);
				igl::png::texture_from_file("brush.png", brush_tex);
				igl::png::texture_from_file("seamer.png", seamer_tex);
				igl::png::texture_from_file("orienter.png", orienter_tex);
				igl::png::texture_from_file("measurer.png", measurer_tex);
				textures_loaded = true;
			}
		}

		/*
		Icon Credits:
		<div>Icons made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
		<div>Icons made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
		<div>Icons made by <a href="https://www.flaticon.com/authors/vaadin" title="Vaadin">Vaadin</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
		<div>Icons made by <a href="https://www.flaticon.com/authors/monkik" title="monkik">monkik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
		<div>Icons made by <a href="https://www.flaticon.com/authors/good-ware" title="Good Ware">Good Ware</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
		<div>Icons made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/"             title="Flaticon">www.flaticon.com</a></div>
		*/
	};
}