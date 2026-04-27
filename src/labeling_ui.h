#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/png/texture_from_file.h>
#include <igl/file_dialog_open.h>

#include <string>

#include "coarse_knit_mesh.h"

namespace hlk {
	class LabelingUI : public igl::opengl::glfw::imgui::ImGuiMenu {
	public:

		//void set_mesh(CoarseKnitMesh m);

		bool load_quad_mesh_file();
		void load_quad_mesh_file(std::string filename);
		// In-memory loader used by the unified UI to hand off the
		// remeshing-stage quad mesh without a disk roundtrip.
		void load_quad_mesh_in_memory(const Eigen::MatrixXd& V_in,
		                              const Eigen::MatrixXi& F_in);
		void init_quad_mesh_display();

		bool has_loaded_mesh() const { return mesh_loaded; }

		void load_generator();
		bool has_generator = false;
		std::string generator_path = "";
		Eigen::MatrixXd gen_coords;
		void generate_variation(std::string args);

		void load_variation(std::string filename);

		bool mouse_down(int button, int modifier);

		bool mouse_up(int button, int modifier);

		bool mouse_move(int mouse_x, int mouse_y);

		void draw_viewer_menu();

		void update_mesh();

		void optimize_geometry();

		void extract_coarse_graph();

		void extract_fine_graph();

		void trace_graph();

		void generate_instructions();

		void save_coarse_knit_mesh(std::string filename);

		void load_coarse_knit_mesh(std::string filename);
		
		void set_layer(int layer, bool on);

		void solve_topology();

		void save_graph(const igl::opengl::ViewerData& data);

	private:

		CoarseKnitMesh M;
		bool mesh_loaded = false;
		int base_index = 0;
		int overlay_index = -1;
		int graph_index = -1;
		int knit_graph_index = -1;
		int traced_graph_index = -1;
		int debug_index = -1;

		int minimizer_timeout = (int) M.minimizer_timeout;

		bool show_mesh = true;
		bool show_graph = true;
		bool show_knit_graph = true;
		bool show_traced_graph = true;
		bool show_debug = true;
		bool planarize = false;

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

		enum ConstraintsMode {
			SHAPE_MODE,
			SIZE_MODE
		};

		// Modes and settings

		Tool current_tool = ORIENTER;
		EraserMode eraser_mode = ERASE_ORIENTATIONS;
		KnitDirection orienter_mode = LOOP;

		ShapingType shaping_brush = DISTRIBUTED;
		ShapingType short_row_brush = NONE;

		ConstraintsMode measurer_mode = SHAPE_MODE;

		std::string eraser_instructions = "CTRL-Click and drag to erase.";
		std::string texturer_instructions = "CTRL-Click and drag to add\ntexture.";
		std::string seamer_instructions = "CTRL-Click and drag to join\nseams.";
		std::string orienter_instructions = "CTRL-Click and drag to set\norientation. Left click for loop,\nright click for yarn.";
		std::string measurer_instructions = "CTRL-Click and drag to add a\nconstraint. Click an outgoing\nedge to add a constraint.\nShift-Click to add separate\nconstraints.";

		std::string instructions = orienter_instructions;

		bool is_dragging = false;
		int dragging_button = (int) igl::opengl::glfw::Viewer::MouseButton::Left;
		int drag_start_side = -1;
		int last_drag_side = -1;

		bool pick_face(int& fid, Eigen::Vector3f& bc);

		int outgoing_side(int fid, const Eigen::Vector3f& bc);

		// Whether to re-run the solver on mouse-up or not
		bool auto_solve;

		// Texture Names - TODO - We probably want a second data structure to hold a database of these
		std::vector<std::string> texture_names{};
		int current_texture = 0;
		bool knitting_textures_loaded = false;

		// Debug Tooltips On
		bool show_debug_tooltip = false;

		// Texture Handles for UI
		bool textures_loaded = false;
		GLuint eraser_tex, brush_tex, seamer_tex, orienter_tex, measurer_tex;
		GLuint eraser_pressed, brush_pressed, seamer_pressed, orienter_pressed, measurer_pressed;

		void load_textures();
		void load_knitting_textures();

		Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;

		bool white_bg = false;

		double last_size_line = -1.0;
		bool use_last_sizing = false;

		int depth = -1;

		bool use_stacked_planner = false;


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