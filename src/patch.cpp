#include "patch.h"

namespace hlk {
	Patch::Patch(std::vector<std::vector<int>> sides, Eigen::MatrixXd corners)
	{
		// Assume Loop_In, Yarn_Out, Loop_Out, Yarn_In order
		std::vector<int> side_counts;
		for (auto& side : sides) {
			int count = 0;
			for (auto& quad_side : side) {
				count += quad_side;
			}
			side_counts.push_back(count);
		}

		// Initially construct the patch with the smaller side (if it exists) on top 
		auto width = side_counts[0] > side_counts[2] ? side_counts[0] : side_counts[2];
		auto height = side_counts[1] > side_counts[3] ? side_counts[1] : side_counts[3];
		auto top_width = side_counts[0] > side_counts[2] ? side_counts[2] : side_counts[0];
		if (side_counts[1] != side_counts[3]) {
			int temp = width;
			width = height;
			height = temp;
			top_width = side_counts[1] > side_counts[3] ? side_counts[3] : side_counts[1];
		}

		int top_out_width = top_width;
		if (top_width == 0) top_width = 1;

		Eigen::MatrixXi nodes(height, width);
		nodes.setOnes();

		std::vector<int> rowWidths(height);
		std::vector<int> rowDeltas(height - 1);

		for (int i = 0; i < height; ++i) {
			rowWidths[i] = (width - top_width)*i / (height - 1 );
		}

		for (int i = 0; i < height - 1; ++i) {
			rowDeltas[i] = rowWidths[i+1] - rowWidths[i];
		}

		// TODO - Determine removal distribution
		// TODO - Remove tops of columns

	}
};