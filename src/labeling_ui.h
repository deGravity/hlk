#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/png/texture_from_file.h>

#include <string>

//#include "coarse_knit_mesh.h"

namespace hlk {
	class LabelingUI : public igl::opengl::glfw::imgui::ImGuiMenu {
	public:

		//void set_mesh(CoarseKnitMesh m);

		void load_mesh(std::string obj_file);

		bool mouse_down(int button, int modifier);

		bool mouse_up(int button, int modifier);

		bool mouse_move(int mouse_x, int mouse_y);

		void draw_viewer_menu();

	private:

		//CoarseKnitMesh M;

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

		// Modes and settings

		Tool current_tool = ERASER;
		EraserMode eraser_mode = ERASE_ORIENTATIONS;

		std::string eraser_instructions = "CTRL-Click and drag to erase.";
		std::string texturer_instructions = "CTRL-Click and drag to add texture.";
		std::string seamer_instructions = "CTRL-Click and drag to join seams.";
		std::string orienter_instructions = "CTRL-CLick and drag to set orientation. Left click for loop, right click for yarn.";
		std::string measurer_instructions = "CTRL-Click and drag to add a constraint. Click an outgoing edge to add a constraint. Shift-Click to add separate constraints.";

		std::string instructions = eraser_instructions;

		bool is_dragging = false;
		igl::opengl::glfw::Viewer::MouseButton dragging_button = igl::opengl::glfw::Viewer::MouseButton::Left;
		

		// Whether to re-run the solver on mouse-up or not
		bool auto_solve;


		// Texture Handles for UI
		bool textures_loaded = false;
		GLuint eraser_tex, brush_tex, seamer_tex, orienter_tex, measurer_tex;
		GLuint eraser_pressed, brush_pressed, seamer_pressed, orienter_pressed, measurer_pressed;

		void load_textures();

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