#pragma once

#include <Eigen/Core>
#include <vector>

namespace hlk {
	struct Grid {
		Eigen::MatrixXi cells;
		std::vector<int> symbols;

		void cells_from_diffs(int rows, int cols, std::vector<std::vector<int>> merge_points);
	};
};