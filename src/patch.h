#pragma once

#include <vector>
#include <Eigen/Core>

#include "knit_graph.h"

namespace hlk {
	struct Patch : public KnitGraph {
		Patch(std::vector<std::vector<int>> sides, Eigen::MatrixXd corners);


	};
};