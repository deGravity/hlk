#include "patch.h"

#include <deque>
#include <queue>
#include <iostream>
#include <fstream>
#include <iterator>
#include <regex>

#include <igl/mvc.h>

namespace hlk {
	
	void Chart::read_from_file(std::string path) {

		setup_stitch_db();

		std::ifstream file(path);
		std::string line;
		while (std::getline(file, line)) {
			std::istringstream line_stream(line);
			std::vector<std::string> stitches(std::istream_iterator<std::string>{line_stream},
				std::istream_iterator<std::string>());

			if (!stitches.empty()) {
				rows.emplace_back();
				for (auto& stitch : stitches) {
					rows.back().push_back(parse_stitch(stitch));
				}
			}
		}
	}

	void Chart::setup_stitch_db()
	{
		stitch_db["k"] = ChartCell();
		stitch_db["k"].has_yarn_in = true;
		stitch_db["k"].has_yarn_out = true;
		stitch_db["k"].num_inputs = 1;
		stitch_db["k"].input_order = std::vector<int>{ 0 };
		stitch_db["k"].outputs = std::vector<LoopType>{ LoopType::KNIT };
		stitch_db["k"].output_signs = std::vector<LoopSign>{ LoopSign::NONE };

		stitch_db["p"] = ChartCell();
		stitch_db["p"].has_yarn_in = true;
		stitch_db["p"].has_yarn_out = true;
		stitch_db["p"].num_inputs = 1;
		stitch_db["p"].input_order = std::vector<int>{ 0 };
		stitch_db["p"].outputs = std::vector<LoopType>{ LoopType::PURL };
		stitch_db["p"].output_signs = std::vector<LoopSign>{ LoopSign::NONE };

		stitch_db["s"] = ChartCell();
		stitch_db["s"].has_yarn_in = true;
		stitch_db["s"].has_yarn_out = true;
		stitch_db["s"].num_inputs = 1;
		stitch_db["s"].input_order = std::vector<int>{ 0 };
		stitch_db["s"].outputs = std::vector<LoopType>{ LoopType::SLIP };
		stitch_db["s"].output_signs = std::vector<LoopSign>{ LoopSign::MINUS };

		stitch_db["m"] = ChartCell();
		stitch_db["m"].has_yarn_in = true;
		stitch_db["m"].has_yarn_out = true;
		stitch_db["m"].num_inputs = 0;
		stitch_db["m"].input_order = std::vector<int>{};
		stitch_db["m"].outputs = std::vector<LoopType>{ LoopType::KNIT };
		stitch_db["m"].output_signs = std::vector<LoopSign>{ LoopSign::NONE };

		stitch_db["py"] = ChartCell();
		stitch_db["py"].has_yarn_in = true;
		stitch_db["py"].has_yarn_out = true;
		stitch_db["py"].num_inputs = 1;
		stitch_db["py"].input_order = std::vector<int>{ 0 };
		stitch_db["py"].outputs = std::vector<LoopType>{ LoopType::PURL, LoopType::YARNOVER };
		stitch_db["py"].output_signs = std::vector<LoopSign>{ LoopSign::NONE, LoopSign::NONE };

		stitch_db["d21k"] = ChartCell();
		stitch_db["d21k"].has_yarn_in = true;
		stitch_db["d21k"].has_yarn_out = true;
		stitch_db["d21k"].num_inputs = 2;
		stitch_db["d21k"].input_order = std::vector<int>{ 1, 0 };
		stitch_db["d21k"].outputs = std::vector<LoopType>{ LoopType::KNIT };
		stitch_db["d21k"].output_signs = std::vector<LoopSign>{ LoopSign::NONE };

		stitch_db["["] = ChartCell();
		stitch_db["["].has_yarn_in = false;
		stitch_db["["].has_yarn_out = true;
		stitch_db["["].num_inputs = 1;
		stitch_db["["].input_order = std::vector<int>{ 0 };
		stitch_db["["].outputs = std::vector<LoopType>{ LoopType::KNIT };
		stitch_db["["].output_signs = std::vector<LoopSign>{ LoopSign::NONE };

		stitch_db["]"] = ChartCell();
		stitch_db["]"].has_yarn_in = true;
		stitch_db["]"].has_yarn_out = false;
		stitch_db["]"].num_inputs = 1;
		stitch_db["]"].input_order = std::vector<int>{ 0 };
		stitch_db["]"].outputs = std::vector<LoopType>{ LoopType::KNIT };
		stitch_db["]"].output_signs = std::vector<LoopSign>{ LoopSign::NONE };
	}

	ChartCell Chart::parse_stitch(const std::string & stitch)
	{
		ChartCell cell;
		return stitch_db[stitch];
	}

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
		std::deque<std::shared_ptr<KnitGraphEdge>> yarn_edges;
		boundaries.resize(4);

		for (int i = 0; i < side_lengths[0]; ++i) {
			auto loop_in_edge = std::make_shared<KnitGraphEdge>();
			loop_edges.push(loop_in_edge);
			graph.edges.push_back(loop_in_edge);
			boundaries[0].push_back(loop_in_edge);
		}

		for (int i = 0; i < side_lengths[3]; ++i) {
			auto yarn_in_edge = std::make_shared<KnitGraphEdge>();
			yarn_in_edge->is_loop = false;
			graph.edges.push_back(yarn_in_edge);
			boundaries[3].push_back(yarn_in_edge);
			yarn_edges.push_front(yarn_in_edge);
		}

		int row_num = -1;
		for (auto& row : chart) {
			++row_num;
			for (int col = 0; col < row.size(); ++col) {
				auto& stitch = row[col];
				auto node = std::make_shared<KnitGraphNode>();
				node->pos = Eigen::RowVector3d(
					(double) (row_num + 1)/(chart.size()+1), 
					(double) (col + 1) /(row.size()+1), 0);
				graph.nodes.push_back(node);
				// Connect Loop-In Edges
				for (int i = 0; i < stitch.num_inputs; ++i) {
					loop_edges.front()->dst = node;
					node->bottom.push_back(loop_edges.front());
					loop_edges.pop();
				}
				node->loop_stacking = stitch.input_order;
				// Create Loop-Out Edges
				for (int i = 0; i < stitch.outputs.size(); ++i) {
					auto out_edge = std::make_shared<KnitGraphEdge>();
					node->top.push_back(out_edge);
					out_edge->type = stitch.outputs[i];
					out_edge->sign = stitch.output_signs[i];
					out_edge->src = node;
					graph.edges.push_back(out_edge);
					loop_edges.push(out_edge);
				}
				// Connect Yarn-In Edge
				if (stitch.has_yarn_in) {
					std::shared_ptr<KnitGraphEdge> yarn_in_edge;
					if (col == 0) {
						yarn_in_edge = yarn_edges.back();
						yarn_edges.pop_back();
					}
					else {
						yarn_in_edge = yarn_edges.front();
						yarn_edges.pop_front();
					}
					node->left = yarn_in_edge;
					yarn_in_edge->dst = node;
				}
				// Create Yarn-Out Edge
				if (stitch.has_yarn_out) {
					auto yarn_out_edge = std::make_shared<KnitGraphEdge>();
					yarn_out_edge->is_loop = false;
					yarn_out_edge->src = node;
					node->right = yarn_out_edge;
					graph.edges.push_back(yarn_out_edge);
					yarn_edges.push_front(yarn_out_edge);
				}
			}
		}

		// Remaining edges are the boundaries
		while (loop_edges.size() > 0) {
			boundaries[2].push_back(loop_edges.front());
			loop_edges.pop();
		}
		while (yarn_edges.size() > 0) {
			boundaries[1].push_back(yarn_edges.back());
			yarn_edges.pop_back();
		}
	}

	void Patch::interpolate_coordinates()
	{
		Eigen::MatrixXd V;
		Eigen::MatrixXi E; // Unused
		graph.edge_list_graph(V, E);

		Eigen::MatrixXd C;
		C.resize(corners.rows(), 2);
		V = V.block(0, 0, V.rows(), 2); // Drop the Z coordinate

		Eigen::MatrixXd W;

		// Create a 2d corner block
		std::vector<std::vector<double>> side_coords(4);

		// TODO - duplicate code from make_graph
		std::vector<int> side_lengths;
		for (auto& side : sides) {
			int accum = 0;
			for (auto& l : side) {
				accum += l;
			}
			side_lengths.push_back(accum);
		}

		for (int s = 0; s < 4; ++s) {
			if (side_lengths[s] == 0) {
				if (sides[s].size() > 0) {

				}
				side_coords[s].push_back(0.0);
				side_coords[s].push_back(1.0);
			}
			else {
				double accum_length = 0.0;
				for (int i = 0; i < sides[s].size(); ++i) {
					side_coords[s].push_back(accum_length);
					double l = (double)sides[s][i] / side_lengths[s];
					accum_length += l;
				}
				side_coords[s].push_back(accum_length);
			}
		}
		// Corner Vertices will be doubled - we need to account for this
		// when interpolating later
		//
		std::vector<Eigen::RowVector2d> coords2d;
		for (double x : side_coords[0]) {
			coords2d.push_back(Eigen::RowVector2d(x, 0.0));
		}
		for (double y : side_coords[1]) {
			coords2d.push_back(Eigen::RowVector2d(1.0, y));
		}
		for (double x : side_coords[2]) {
			coords2d.push_back(Eigen::RowVector2d(1-x, 0.0));
		}
		for (double y : side_coords[3]) {
			coords2d.push_back(Eigen::RowVector2d(0.0, 1 - y));
		}

		C.resize(coords2d.size(), 2);
		for (int i = 0; i < coords2d.size(); ++i) {
			C.row(i) = coords2d[i];
		}

		igl::mvc(V, C, W);

		Eigen::MatrixXd C3d;

		Eigen::MatrixXd node_positions = W * C3d;
	}


	/*
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
	*/
};