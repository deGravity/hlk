#pragma once

#include <Eigen/Core>

#include <vector>
#include <memory>
#include <utility>

#include "patch.h"
#include "coarse_knit_mesh.h"

// The Dual-Graph of a Coarse-Knit-Mesh, after all labeling and numbering

namespace hlk {

	struct CoarseKnitGraph {
		struct Node {
			Eigen::MatrixXd corners;
			std::vector<int> side_stitches;
			std::vector < std::vector<std::pair<std::shared_ptr<Node>, int>>> neighbors; // Generalized_sides->Neighbors->Neighbor,back_idx
			std::shared_ptr<Patch> patch;
			std::vector<bool> connected;
			std::vector<std::vector<int>> sides;


			void init_patch();
		};

		struct Edge {
			bool is_loop;
			int src;
			int dst;
			int src_side;
			int dst_side;
		};

		std::vector<Node> nodes;

		CoarseKnitGraph(const CoarseKnitMesh& mesh);

		void init_patches();
		void connect_patches();
	};

	
};