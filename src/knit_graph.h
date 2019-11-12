#pragma once

#include <eigen/Core>

namespace hlk {

	struct KnitGraphEdge {
		bool is_loop;
		std::shared_ptr<KnitGraphNode> neighbor;
	};

	struct KnitGraphNode {
		bool purl[2];
		bool fixed;
		Eigen::RowVector3d pos;

	};

	struct KnitGraph {

	};
};