#pragma once

#include <Eigen/Core>

#include <vector>
#include <memory>
#include <utility>

#include "patch.h"

// The Dual-Graph of a Coarse-Knit-Mesh, after all labeling and numbering

namespace hlk {

	struct CoarseKnitGraph {

		struct Edge {
			bool is_loop;
			int src;
			int dst;
			int src_side;
			int dst_side;
		};

		std::vector<Patch> patches;
		std::vector<Edge> edges;

	};

	
};