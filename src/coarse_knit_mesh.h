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
		IN,
		OUT
	};

	enum ShapingType
	{
		NONE,
		IN_SIDE,
		OUT_SIDE,
		BOTH_SIDES, // Only Valid for Inc/Dec
		DISTRIBUTED // Only Valid for Inc/Dec
	};



	// TODO - These definitions are mostly just copied from the old code
	// a better detangling of the mesh structure from the dual-graph
	// structure could be very useful.
	// For instance, we don't really need to completely duplicate the
	// linking structure since the mesh class allows us to query all
	// relevant neighbors


	struct CoarseKnitEdge {
		CoarseKnitEdge(Optimizer& opt, double len, int i, CoarseKnitMesh* m);
		std::shared_ptr<BoolProp> is_seam;
		z3::expr seam_count;
		std::shared_ptr<CoarseKnitSide> a;
		std::shared_ptr<CoarseKnitSide> b;
		int index;
		CoarseKnitMesh* mesh;
		void print_info(std::string line_prefix);

		double length;
		std::shared_ptr<IntProp> num_stitches;
		std::shared_ptr<z3::expr> size_error;

		void save_labeling(std::ofstream& f);
		void load_labeling(std::ifstream& f);
		void save_sizing(std::ofstream& f);
		void load_sizing(std::ifstream& f);
	};

	struct CoarseKnitQuad {
		
		CoarseKnitQuad(Optimizer& opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<IntProp> time;
		std::shared_ptr<z3::expr> orientation;
		ShapingType shaping_distribution;
		ShapingType short_row_distribution;
		std::vector<std::shared_ptr<CoarseKnitEdge>> sides;
		int index;
		CoarseKnitMesh* mesh;
		void setup_orientation();
		void print_info(std::string line_prefix);
		Eigen::Matrix<bool, -1, -1> texture;
		Eigen::Vector3d texture_color;
		Eigen::Vector2i texture_uv;
		Eigen::MatrixXd texture_coords;
		bool orientation_sol;
		Eigen::Vector2i texture_size;
		void InitTextureSize();

		z3::expr is_normal();

		bool is_splittable = false;

		void save_labeling(std::ofstream& f);
		void load_labeling(std::ifstream& f);
	};

	struct CoarseKnitSide {
		CoarseKnitSide(Optimizer& opt, int i, CoarseKnitMesh* m);
		std::shared_ptr<BoolProp> is_loop;
		std::shared_ptr<BoolProp> is_out;
		std::shared_ptr<IntProp> time;
		bool is_border;
		std::shared_ptr<CoarseKnitEdge> edge;
		std::shared_ptr<CoarseKnitSide> opposite;
		std::shared_ptr<CoarseKnitQuad> face;
		int index;
		CoarseKnitMesh* mesh;
		void print_info(std::string line_prefix);

		std::shared_ptr<IntProp> num_stitches;

		void save_labeling(std::ofstream& f);
		void load_labeling(std::ifstream& f);
		void save_sizing(std::ofstream& f);
		void load_sizing(std::ifstream& f);
	};

	struct CoarseKnitMesh : LabeledQuadMesh {
		
		std::vector<std::vector<int>> seams;
		std::vector<std::vector<int>> size_lines;
		std::vector<std::vector<int>> symmetries;

		std::vector<CoarseKnitEdge> edges;
		std::vector<CoarseKnitQuad> quads;
		std::vector<CoarseKnitSide> sides;

		// Copy shaping between quads
		void copy_shaping(int origin_side, int dest_side);
		void set_texture(int side, int texture);

		void seam_off(int side);
		void seam_on(int side);
		void toggle_seam(int side);
		void join_seam(int side_a, int side_b);

		// Split any seams in the seam list that cross this vertex
		void split_seams(int vertex);

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


		// Construct Linked Structure
		// Find base complex seams and initialize
		// Initialize Constraints
		// Set Initial Textures
		virtual void init();
	};
}