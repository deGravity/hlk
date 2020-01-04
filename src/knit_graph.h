#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>

#include "autoknit.h"
#include "vkmp/xferplan/Stitch.hpp"

namespace hlk {

	struct KnitGraphEdge;
	struct KnitGraphNode;

	enum class LoopType {
		KNIT,
		PURL,
		SLIP,
		YARNOVER,
	};

	enum class LoopSign {
		PLUS,
		MINUS,
		NONE
	};

	struct KnitGraphEdge {
		std::shared_ptr<KnitGraphNode> src;
		std::shared_ptr<KnitGraphNode> dst;

		bool is_loop = true;

		// These fields are only used by loop edges
		LoopType type;
		LoopSign sign;

		bool contracted = false; // Flag that edge has been contracted away and should be removed from edge lists
	};

	// Add Stitch types here
	// Format:
	//         id, "name", is_basic_type, is_same_bed=0, is_custom_xfer, in_end=false
	namespace STITCH {
		static vkmp::StData YARNEND = { 0, "yarnend", 0, 0, 0, false };
		static vkmp::StData KNIT = { 1, "knit", 0, 0, 0, false };
		static vkmp::StData TUCK = { 2, "tuck", 0, 0, 0, false };
		static vkmp::StData MISS = { 3, "miss", 0, 0, 0, false };
		static vkmp::StData DECREASE = { 4, "decrease", 0, 0, 0, false }; // Identical to knit
		static vkmp::StData DEC_L = { 5, "dec_l", 1, 0, 1, false };
		static vkmp::StData DEC_R = { 6, "dec_r", 1, 0, 1, false };
		static vkmp::StData INCREASE = { 7, "increase", 0, 0, 0, false };
		static vkmp::StData INCREASE_SPLIT = { 8, "increase_split", 0, 0, 0, false };
		static vkmp::StData INCREASE_L = { 9, "increase_l", 0, 0, 0, false };
		static vkmp::StData INCREASE_R = { 10, "increase_r", 0, 0, 0, false };
		static vkmp::StData INCREASE_SPLIT_L = { 11, "increase_split_l", 0, 0, 0, false };
		static vkmp::StData INCREASE_SPLIT_R = { 12, "increase_split_r", 0, 0, 0, false };
		static vkmp::StData SHORTROW = { 13, "shortrow", 0, 0, 0, false };
		static vkmp::StData CSHORT = { 14, "cshort", 0, 0, 0, false };
		static vkmp::StData PURL = { 15, "purl", 0, 0, 0, false };
		static vkmp::StData KNITPLATED = { 16, "knitPlated", 0, 0, 0, false };
		static vkmp::StData CABLE2 = { 17, "cable2", 1, 0, 0, false };
		static vkmp::StData CABLE2B = { 18, "cable2b", 1, 0, 0 };
		static vkmp::StData CABLE3_012 = { 19, "cable3_012", 1, 0, 0, false };
		static vkmp::StData CABLE3_021 = { 20, "cable3_021", 1, 0, 0, false };
		static vkmp::StData CABLE3_102 = { 21, "cable3_102", 1, 0, 0, false };
		static vkmp::StData CABLE3_120 = { 22, "cable3_120", 1, 0, 0, false };
		static vkmp::StData CABLE3_201 = { 23, "cable3_201", 1, 0, 0, false };
		static vkmp::StData CABLE3_210 = { 24, "cable3_210", 1, 0, 0, false };
		static vkmp::StData CABLE4_0123 = { 25, "cable4_0123", 1, 0, 0, false };
		static vkmp::StData CABLE4_2301 = { 26, "cable4_2301", 1, 0, 0, false };
		static vkmp::StData BINDOFF = { 27, "bindoff", 1, 0, 0, false };
		static vkmp::StData CASTON = { 28, "caston", 0, 0, 0, false };
		static vkmp::StData FAIR_A = { 28, "fair_a", 0, 0, 0, false };
		static vkmp::StData FAIR_B = { 29, "fair_b", 0, 0, 0, false };
		static vkmp::StData FAIR_AT = { 30, "fair_at", 0, 0, 0, false };
		static vkmp::StData FAIR_BT = { 31, "fair_bt", 0, 0, 0, false };
	};

	struct KnitGraphNode {
		std::vector<std::shared_ptr<KnitGraphEdge>> top;
		std::vector<std::shared_ptr<KnitGraphEdge>> bottom;
		std::shared_ptr<KnitGraphEdge> left;
		std::shared_ptr<KnitGraphEdge> right;
		std::vector<int> loop_stacking;

		Eigen::RowVector3d pos;

		int index = -1; // Useful for some algorithms
		
		bool contractable(); // If true, this is a pass-through node that should be removed
		bool contract();
		bool contracted = false; // If true, this should be removed from node lists

		vkmp::StData get_data(bool first_tracing);

	};

	struct KnitGraph {
		std::vector<std::shared_ptr<KnitGraphNode>> nodes;
		std::vector<std::shared_ptr<KnitGraphEdge>> edges;

		bool doubled_wales = true; // Whether or not each node is this graph represents two stitches in the final objec
		bool contracted = false; // Whether or not all contractible nodes have been removed
		bool doubled = true; // Whethor or not all nodes represent two stitches
		bool scheduled = false; // Whether or not this graph has been scheduled and shift-paths created

		void contract(); // Contract and remove all contractible edges
		void split_doubled(); // Split all nodes into two loop-wise connected nodes - pre-req. for scheduling
		void schedule(); // Order and assign yarns for each node. Fix yarn in/out and create shift-paths

		void trace(std::string filename);
		ak::RowColGraph make_row_col_graph();

		void re_index(); // Assign each node a unique index number
		void edge_list_graph(Eigen::MatrixXd& V, Eigen::MatrixXi& E);

		void build_mesh(double th, int poly_size,
			Eigen::MatrixXd& V,
			Eigen::MatrixXi& F,
			Eigen::MatrixXd& C);
	};
};