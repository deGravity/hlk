#pragma once

#include <Eigen/Core>
#include <vector>

#include "autoknit.h"

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