#pragma once

#include "labeled_quad_mesh.h"
#include "optimizer.h"

#include <memory>
#include <vector>

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
		// Data We Definitely Want
		CoarseKnitEdge(Optimizer& topo_opt, Optimizer& geo_opt, int i, CoarseKnitMesh* m);
		int seam;
		int index;
		CoarseKnitMesh* mesh;

		std::vector<std::pair<z3::expr, std::string>> get_topology_constraints();
		std::vector<std::pair<z3::expr, std::string>> get_geometry_constraints();
		void update_texture();
	};

	struct CoarseKnitQuad {
		
		CoarseKnitQuad(Optimizer& topo_opt, Optimizer& geo_opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<IntProp> time;
		int index;
		CoarseKnitMesh* mesh;

		ShapingType shaping_distribution;
		ShapingType short_row_distribution;

		int texture_id = -1;

		std::vector<std::pair<z3::expr, std::string>> get_topology_constraints();
		std::vector<std::pair<z3::expr, std::string>> get_geometry_constraints();

		void update_texture();
	};

	struct CoarseKnitSide {
		CoarseKnitSide(Optimizer& topo_opt, Optimizer& geo_opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<BoolProp> is_loop;
		std::shared_ptr<BoolProp> is_out;
		std::shared_ptr<IntProp> stitches;
		int index;
		CoarseKnitMesh* mesh;

		std::vector<std::pair<z3::expr, std::string>> get_topology_constraints();
		std::vector<std::pair<z3::expr, std::string>> get_geometry_constraints();

		void update_texture();
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
		std::vector<std::vector<int>> size_lines;
		std::vector<std::vector<int>> symmetries;

		std::vector<CoarseKnitEdge> edges;
		std::vector<CoarseKnitQuad> quads;
		std::vector<CoarseKnitSide> sides;

		bool optimize_topology();
		bool optimize_geometry();

		// Copy shaping between quads
		void copy_shaping(int origin_side, int dest_side);
		void set_texture(int side, int texture);

		void seam_off(int side);
		void seam_on(int side);
		void toggle_seam(int side);
		//void join_seam(int side_a, int side_b);

		void add_seam(std::vector<int> sides);

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

		// Construct Linked Structure
		// Find base complex seams and initialize
		// Initialize Constraints
		// Set Initial Textures
		virtual void init();

		double scale = 1; // Units:Inches
		double stitch_guage = 7.0; // In stitches / inch 
		double row_guage = 14.0; // In rows / inch


		// TODO - be better than this
		int min_time = 0;
		int max_time = 1;

	};
}