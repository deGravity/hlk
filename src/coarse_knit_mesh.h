#pragma once

#include "labeled_quad_mesh.h"
#include "optimizer.h"
#include "coarse_knit_graph.h"

#include <igl/opengl/ViewerData.h>

#include <memory>
#include <vector>
#include <fstream>

namespace hlk {

	enum KnitDirection {
		YARN,
		LOOP
	};

	enum KnitOrientation {
		KNIT_IN,
		KNIT_OUT
	};

	enum ShapingType
	{
		NONE,
		IN_SIDE,
		OUT_SIDE,
		BOTH_SIDES, // Only Valid for Inc/Dec
		DISTRIBUTED // Only Valid for Inc/Dec
	};

	// Forward Declarations
	struct CoarseKnitEdge; 
	struct CoarseKnitMesh;
	struct CoarseKnitQuad;
	struct CoarseKnitSide;

	struct CoarseKnitEdge {
		CoarseKnitEdge(Optimizer& topo_opt, Optimizer& geo_opt, int i, CoarseKnitMesh* m);
		int seam;
		int index;
		CoarseKnitMesh* mesh;

		std::vector<std::pair<z3::expr, std::string>> get_topology_constraints();
		std::vector<std::pair<z3::expr, std::string>> get_geometry_constraints();

		int target_stitch_count();
		
		void update_texture();

		void save(std::ofstream& f);
		void load(std::ifstream& f);
	};

	struct CoarseKnitQuad {
		
		CoarseKnitQuad(Optimizer& topo_opt, Optimizer& geo_opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<IntProp> time;
		int index;
		CoarseKnitMesh* mesh;

		ShapingType shaping_distribution;
		ShapingType short_row_distribution;

		int texture_id = 0;

		std::vector<std::vector<int>> get_symmetries();

		std::vector<std::pair<z3::expr, std::string>> get_topology_constraints();
		std::vector<std::pair<z3::expr, std::string>> get_geometry_constraints();

		void get_corners(Eigen::MatrixXd& C) const;
		void get_generalized_corners(Eigen::MatrixXd& C) const;
		int get_generalized_bottom_left() const;
		void get_sides_stitches(std::vector<std::vector<int>>& sides) const;
		int to_generalized(int side) const;

		void update_texture();

		void save(std::ofstream& f);
		void load(std::ifstream& f);
	};

	struct CoarseKnitSide {
		CoarseKnitSide(Optimizer& topo_opt, Optimizer& geo_opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<BoolProp> is_loop;
		std::shared_ptr<BoolProp> is_out;
		std::shared_ptr<IntProp> stitches;
		std::shared_ptr<IntProp> stitches_backup; // Used for caching for symmetry optimization
		double get_gauge();
		void cache_stitches();
		void uncache_stitches();
		bool is_representative = true; // If this is a representative of a symmetry
		int index;
		CoarseKnitMesh* mesh;

		int target_stitch_count();

		std::vector<std::pair<z3::expr, std::string>> get_topology_constraints();
		std::vector<std::pair<z3::expr, std::string>> get_geometry_constraints();

		z3::expr get_geometry_cost();

		void update_texture();

		int generalized_index();

		void save(std::ofstream& f);
		void load(std::ifstream& f);
	};

	struct CoarseKnitMesh : LabeledQuadMesh {

		// These must remain first since they must be destructed last!
		hlk::Optimizer topology_optimizer;
		hlk::Optimizer geometry_optimizer;
		
		std::vector<std::shared_ptr<BoolProp>> seams;
		// Edges in a seam
		// Will always be ordered 
		std::vector<std::vector<int>> seam_edges;
		std::vector<bool> vertex_in_seam;
		std::vector<std::pair<std::vector<int>, double>> size_lines;
		std::vector<std::vector<int>> symmetries;

		std::vector<double> side_lengths;

		std::vector<bool> vertex_is_special;

		std::vector<CoarseKnitEdge> edges;
		std::vector<CoarseKnitQuad> quads;
		std::vector<CoarseKnitSide> sides;

		bool optimize_topology();
		bool optimize_geometry();

		CoarseKnitGraph get_dual();

		// Get Constraints for seams
		std::vector<std::pair<z3::expr, std::string>> get_seam_costs();

		void add_size_line(std::vector<int> line_sides);

		// Get Constraints for line sizes
		std::vector<std::pair<z3::expr, std::string>> size_line_constraints();

		// Get Symmetry constraints
		std::vector<std::pair<z3::expr, std::string>> get_symmetry_constraints();

		// Copy shaping between quads
		void copy_shaping(int origin_side, int dest_side);
		void set_shaping(int side, ShapingType shaping, ShapingType short_row);
		void set_texture(int side, int texture);

		void seam_off(int side);
		void seam_on(int side);
		void toggle_seam(int side);
		//void join_seam(int side_a, int side_b);

		void toggle_special_vertex(int vertex);

		void add_seam(std::vector<int> sides);
		// Get the line that a seam would trace
		std::vector<int> trace_seam(int side);

		void debug_label_vertices(igl::opengl::ViewerData* debug);
		void debug_label_vertex(igl::opengl::ViewerData* debug, int v);
		void debug_label_edges(igl::opengl::ViewerData* debug);
		void debug_label_edge(igl::opengl::ViewerData* debug, int e);
		void debug_label_sides(igl::opengl::ViewerData* debug);
		void debug_label_side(igl::opengl::ViewerData* debug, int s);

		// Split any seams in the seam list that cross this vertex
		void split_seams(int vertex_a, int vertex_b);
		void split_seam(int seam, std::vector<int> vertices);

		// Direction = Loop / Yarn
		// Orientation = In / Out
		// Convention is "Is Loop" and "Is Out"
		// so  loop = true, yarn = false
		// and out = true, in = false

		void toggle_orientation(int side);
		void toggle_direction(int side);
		// Dragging across an edge indicates direction from out to in
		void paint_direction(int side_out, int side_in, KnitDirection dir);


		void erase_seam(int side);
		void erase_orientation(int side);
		void erase_textue(int side);
		void erase_shaping(int side);

		void update_textures();

		virtual void save(std::ofstream& f);
		virtual void load(std::ifstream& f);

		// Construct Linked Structure
		// Find base complex seams and initialize
		// Initialize Constraints (optional)
		// Set Initial Textures
		virtual void init();
		virtual void init(bool do_opt = true);

		double scale = 1; // In Units / Inches
		double stitch_gauge = 6.67; // In stitches / inch
		double row_gauge = 4.16; // In rows / inch

		double tolerance = 0.01;

		int edge_tolerance = 0;
		int course_tolerance = 0;
		int wale_tolerance = 0;
		int critical_tolerance = 0;


		// TODO - be better than this
		int min_time = 0;
		int max_time = 1;

		unsigned int minimizer_timeout = 1;

		bool topology_solved = false;
		bool geometry_solved = false;

		bool symmetrize = true;
	};
}