#include "patch.h"

#include <deque>
#include <queue>

namespace hlk {


	void make_left_dec(int bottom, int top, int height) {

		std::vector<int> rowWidths(height);
		std::vector<int> rowDeltas(height - 1);

		for (int i = 0; i < height; ++i) {
			rowWidths[i] = (bottom - top) * i / (height - 1);
		}

		for (int i = 0; i < height - 1; ++i) {
			rowDeltas[i] = rowWidths[i + 1] - rowWidths[i];
		}
	}

	void Patch::make_graph(std::vector<std::vector<ChartCell>> chart)
	{
		std::vector<int> side_lengths;
		for (auto& side : sides) {
			int accum = 0;
			for (auto& l : side) {
				accum += l;
			}
			side_lengths.push_back(accum);
		}
		std::queue<std::shared_ptr<KnitGraphEdge>> loop_edges;
		std::deque<KnitGraphEdge> yarn_edges;
		boundaries.resize(4);

		for (int i = 0; i < side_lengths[0]; ++i) {
			loop_edges.push(std::make_shared<KnitGraphEdge>());
		}

		for (auto& row : chart) {

		}
		
	}

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

		// By Default right is yarn out and up is loop out
		bool up_is_out = true;
		bool right_is_out = true;
		bool up_is_loop = true;
		bool right_is_loop = false;


		// Figure out which direction and orientation right and up are
		// Rotate default orientation so smaller side is up

		if (side_counts[1] != side_counts[3]) { // up is a yarn instead of a loop
			up_is_loop = false;
			right_is_loop = true;

			int temp = width;
			width = height;
			height = temp;
			top_width = side_counts[1] > side_counts[3] ? side_counts[3] : side_counts[1];
			if (side_counts[1] > side_counts[3]) {
				up_is_out = false;
				right_is_out = true;
			}
			else {
				up_is_out = true;
				right_is_out = false;
			}
		}
		else {
			if (side_counts[0] < side_counts[2]) {
				// Top is bigger so we are rotated 180 deg. from default
				up_is_out = false;
				right_is_out = false;
			}
		}



		int top_out_width = top_width;
		if (top_width == 0) top_width = 1;

		Eigen::MatrixXi chart = Eigen::MatrixXi::Zero(height, width);

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

		// For now, just have left side right leaning increases

		std::vector<std::vector<int>> row_charts;
		for (int i = 0; i < height; ++i) {
			row_charts.push_back(std::vector<int>(rowWidths[i], 0));
			if (i < height - 1) {
				// Max degree 2
				assert(2 * rowDeltas[i] <= rowWidths[i]);
				for (int j = 0; j < rowDeltas[i]; ++j) {
					row_charts.back()[2 * j] = 1;
				}
			}
		}

		const int EMPTY = -2;

		// Fill in the main chart
		for (int i = 0; i < height; ++i) {
			int k = 0;
			for (int j = 0; j < width; ++j) {
				if (chart(i, j) != EMPTY) {
					chart(i, j) = row_charts[i][k];
					if (row_charts[i][k] != 0) {
						for (int l = i + 1; l < height; ++l) {
							chart(l, j) = EMPTY;
						}
					}
					++k;
					// Not doing an overflow bounds check here because failing 
					// overflow means there is an error in row allocation
				}
				assert(k == rowWidths[i]); // Assert that we placed every stitch
			}
		}

		const int BOUNDARY = -3;

		// Build a chart that includes the boundaries
		Eigen::MatrixXi full_chart(height + 2, width + 2);
		full_chart.setConstant(BOUNDARY);
		full_chart(0, 0) = EMPTY;
		full_chart(0, width + 1) = EMPTY;
		full_chart(height + 1, 0) = EMPTY;
		full_chart(height + 1, width + 1) = EMPTY;
		for (int i = 0; i < width; ++i) {
			if (chart(height - 1, i) == EMPTY) {
				full_chart(height + 1, i) == EMPTY;
			}
		}
		full_chart.block(1, 1, height, width) = chart;

		// Now Build a Matrix of smart pointers that we'll use to hook everything up

		Eigen::Matrix<std::shared_ptr<KnitGraphNode>, Eigen::Dynamic, Eigen::Dynamic> nodes(height + 2, width + 2);
		// Initialize the non-empty nodes
		for (int i = 0; i < full_chart.rows(); ++i) {
			for (int j = 0; j < full_chart.cols(); ++j) {
				if (full_chart(i, j) != EMPTY) {
					nodes(i, j) = std::make_shared<KnitGraphNode>();
				}
			}
		}

		auto attach = [&](int i1, int j1, int i2, int j2, bool is_out, bool is_loop) {
			auto& A = nodes(i1, j1);
			auto& B = nodes(i2, j2);
			auto& src = is_out ? A : B;
			auto& dst = is_out ? B : A;


		};

		for (int i = 0; i < full_chart.rows() - 1; ++i) {
			for (int j = 0; j < full_chart.cols() - 1; ++j) {
				int node_type = full_chart(i, j);
				switch (node_type) {
				case EMPTY:
					break; // Nothing to do if no node
				case BOUNDARY:
					
					break;
				case 0:
					break;
				case -1:
					break;
				case 1:
					break;
				default:
					break;
				}

					// TODO - Other cases we don't yet use
					// Consider moving this ne

				}
			}
		}
	}
};