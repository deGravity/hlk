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

	enum ShapingType {
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
		CoarseKnitEdge(Optimizer& geo_opt, Optimizer& topo_opt, int i, CoarseKnitMesh* m);
		int seam;
		int index;
		CoarseKnitMesh* mesh;

		std::vector<std::pair<z3::expr, std::string>> get_constraints();
		void update_texture();
	};

	struct CoarseKnitQuad {
		
		CoarseKnitQuad(Optimizer& geo_opt, Optimizer& topo_opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<IntProp> time;
		int index;
		CoarseKnitMesh* mesh;

		ShapingType shaping_distribution;
		ShapingType short_row_distribution;

		int texture_id = -1;

		// There are no quad-specific geometry constraints
		//std::vector<z3::expr> get_constraints();
		void update_texture();
	};

	struct CoarseKnitSide {
		CoarseKnitSide(Optimizer& geo_opt, Optimizer& topo_opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<BoolProp> is_loop;
		std::shared_ptr<BoolProp> is_out;
		int index;
		CoarseKnitMesh* mesh;

		std::vector<std::pair<z3::expr, std::string>> get_constraints();
		void update_texture();
	};

	struct CoarseKnitMesh : LabeledQuadMesh {
		
		std::vector<std::shared_ptr<BoolProp>> seams;
		std::vector<std::vector<int>> size_lines;
		std::vector<std::vector<int>> symmetries;

		std::vector<CoarseKnitEdge> edges;
		std::vector<CoarseKnitQuad> quads;
		std::vector<CoarseKnitSide> sides;

		bool optimize_geometry();

		// Copy shaping between quads
		void copy_shaping(int origin_side, int dest_side);
		void set_texture(int side, int texture);

		void seam_off(int side);
		void seam_on(int side);
		void toggle_seam(int side);
		//void join_seam(int side_a, int side_b);

		// Split any seams in the seam list that cross this vertex
		//void split_seams(int vertex);

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

		hlk::Optimizer topology_optimizer;
		hlk::Optimizer geometry_optimizer;

	};
}
