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

	std::vector<int> Chart::interpolate(int first, int last, int rows)
	{
		std::vector<int> row_widths(rows);

		assert(rows > 1 || (rows == 1 && first == last)); // If there's only one row, you can't have a size change

		if (rows == 1) {
			return std::vector<int>{first};
		}

		for (int i = 0; i < rows; ++i) {
			row_widths[i] = first + (last - first) * i / (rows - 1);
		}

		return row_widths;
	}

	std::vector<int> Chart::row_diffs(const std::vector<int>& rows)
	{
		std::vector<int> row_diffs(rows.size() - 1);
		for (int i = 0; i < rows.size() - 1; ++i) {
			row_diffs[i] = rows[i + 1] - rows[i];
		}
		return row_diffs;
	}

	// TODO - DRY with other chart generation functions
	void Chart::increases_leaning(int bottom, int top, int height, int dir)
	{
		bool from_nothing = bottom == 0;
		if (bottom == 0) bottom = 1;
		std::vector<int> row_widths = interpolate(bottom, top, height);
		auto diffs = row_diffs(row_widths);
		std::vector<std::vector<int>> num_outputs(height);
		num_outputs[height-1] = std::vector<int>(top, 1);
		for (int i = 0; i < height -1 ; ++i) {
			int num_increases = diffs[i];
			int left_increases = 0;
			int right_increases = 0;
			if (dir < 0) { // left leaning
				left_increases = num_increases;
			}
			else if (dir > 0) { // right leaning
				right_increases = num_increases;
			}
			else { // evenly distributed
				left_increases = ceil((double)num_increases / 2);
				right_increases = floor((double)num_increases / 2);
			}
			int num_straight = row_widths[i] - num_increases;
			for (int j = 0; j < left_increases; ++j) {
				num_outputs[i].push_back(2);
			}
			for (int j = 0; j < num_straight; ++j) {
				num_outputs[i].push_back(1);
			}
			for (int j = 0; j < right_increases; ++j) {
				num_outputs[i].push_back(2);
			}
		}

		rows.resize(height);

		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < row_widths[i]; ++j) {
				ChartCell stitch;
				int n_out = num_outputs[i][j];
				stitch.has_yarn_in = true;
				stitch.has_yarn_out = true;
				stitch.num_inputs = 1;
				stitch.input_order = std::vector<int>{ 0 };
				if (n_out == 1) {
					stitch.outputs = std::vector<LoopType>{ LoopType::KNIT };
					stitch.output_signs = std::vector<LoopSign>{ LoopSign::NONE };
				}
				else {
					stitch.outputs = std::vector<LoopType>{ LoopType::KNIT, LoopType::KNIT };
					stitch.output_signs = std::vector<LoopSign>{ LoopSign::NONE, LoopSign::NONE };
				}

				rows[i].push_back(stitch);
			}
		}

		// Special case for increases from nothing
		if (from_nothing) {
			rows[0][0].num_inputs = 0;
			rows[0][0].input_order = std::vector<int>{};
		}
	}

	void Chart::short_rows(int left, int right, int width, int dir)
	{
		bool from_nothing = left == 0;
		bool to_nothing = right == 0;
		if (from_nothing) left = 1;
		if (to_nothing) right = 1;

		assert(abs(left - right) < width); // Only allow one extra row at a time
		int max_height = left > right ? left : right;
		auto col_heights = interpolate(left, right, width);
		std::vector<std::vector<bool>> cols;
		for (auto height : col_heights) {
			std::vector<bool> col;
			if (dir < 0) { // Short rows Down
				col = std::vector<bool>(height, true);
				col.resize(max_height, false);
			}
			else {
				col = std::vector<bool>(max_height - height, false);
				col.resize(max_height, true);
			}
			cols.push_back(col);
		}

		for (int i = 0; i < max_height; ++i) {
			std::vector<ChartCell> row;
			for (int j = 0; j < width; ++j) {
				if (cols[j][i]) { // Non pass-through cell
					ChartCell stitch;
					stitch.num_inputs = 1;
					stitch.input_order = std::vector<int>{ 0 };
					if (i < max_height - 1 && !cols[j][i+1]) {
						stitch.outputs = std::vector<LoopType>{LoopType::SLIP};
						stitch.output_signs = std::vector<LoopSign>{LoopSign::NONE};
					}
					else {
						stitch.outputs = std::vector<LoopType>{ LoopType::KNIT };
						stitch.output_signs = std::vector<LoopSign>{ LoopSign::NONE };
					}
					if (j > 0 && !cols[j - 1][i]) {
						stitch.has_yarn_in = false;
					}
					else {
						stitch.has_yarn_in = true;
					}

					if (j < width - 1 && !cols[j + 1][i]) {
						stitch.has_yarn_out = false;
					}
					else {
						stitch.has_yarn_out = true;
					}
					row.push_back(stitch);
				}
				else { // Pass-through cell
					ChartCell stitch;
					stitch.num_inputs = 1;
					stitch.input_order = std::vector<int>{ 0 };
					stitch.outputs = std::vector<LoopType>{ LoopType::SLIP };
					stitch.output_signs = std::vector<LoopSign>{ LoopSign::PLUS };
					stitch.has_yarn_in = false;
					stitch.has_yarn_out = false;
					stitch.pass_through = true;
					row.push_back(stitch);
				}
			}
			rows.push_back(row);
		}

		// Remove extraneous output edges
		if (from_nothing) {
			if (dir < 0) {
				rows.front().front().has_yarn_in = false;
				rows.front().front().outputs = std::vector<LoopType>{ LoopType::SLIP };
				rows.front().front().output_signs = std::vector<LoopSign>{ LoopSign::NONE };
			}
			else {
				rows.back().front().has_yarn_in = false;
				rows.back().front().outputs = std::vector<LoopType>{ LoopType::SLIP };
				rows.back().front().output_signs = std::vector<LoopSign>{ LoopSign::NONE };
			}
		}
		if (to_nothing) {
			if (dir < 0) {
				rows.front().back().has_yarn_out = false;
				rows.front().back().outputs = std::vector<LoopType>{ LoopType::SLIP };
				rows.front().back().output_signs = std::vector<LoopSign>{ LoopSign::NONE };
			}
			else {
				rows.back().back().has_yarn_out = false;
				rows.back().back().outputs = std::vector<LoopType>{ LoopType::SLIP };
				rows.back().back().output_signs = std::vector<LoopSign>{ LoopSign::NONE };
			}
		}
	}

	void Chart::decreases_leaning(int bottom, int top, int height, int dir)
	{
		// Special case of a decreases to nothing - pretend there's a stitch there
		// for now. Later we will change these to decreases into the next row or
		// bind-offs, depending on the patch neighbors
		bool to_nothing = top == 0;
		if (to_nothing) top = 1;

		std::vector<int> row_widths = interpolate(bottom, top, height);
		auto diffs = row_diffs(row_widths);
		std::vector<std::vector<int>> num_inputs(height);
		num_inputs[0] = std::vector<int>(bottom, 1);
		for (int i = 1; i < height; ++i) {
			int num_decreases = -diffs[i - 1];
			int left_decreases = 0;
			int right_decreases = 0;
			if (dir < 0) { // left leaning
				left_decreases = num_decreases;
			}
			else if (dir > 0) { // right leaning
				right_decreases = num_decreases;
			}
			else { // evenly distributed
				left_decreases = ceil((double)num_decreases / 2);
				right_decreases = floor((double)num_decreases / 2);
				if (i % 2 == 0) { // swap error per row so we don't purely lean one-way
					int temp = left_decreases;
					left_decreases = right_decreases;
					right_decreases = temp;
				}
			}
			int num_straight = row_widths[i] - num_decreases;
			for (int j = 0; j < left_decreases; ++j) {
				num_inputs[i].push_back(2);
			}
			for (int j = 0; j < num_straight; ++j) {
				num_inputs[i].push_back(1);
			}
			for (int j = 0; j < right_decreases; ++j) {
				num_inputs[i].push_back(2);
			}
		}

		rows.resize(height);

		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < row_widths[i]; ++j) {
				ChartCell stitch;
				stitch.has_yarn_in = true;
				stitch.has_yarn_out = true;
				stitch.num_inputs = num_inputs[i][j];
				if (stitch.num_inputs == 1) {
					stitch.input_order = std::vector<int>{ 0 };
				}
				else {
					// Todo - this is going to affect learning direction
					stitch.input_order = std::vector<int>{ 0,1 }; 
				}
				stitch.outputs = std::vector<LoopType>{ LoopType::KNIT };
				stitch.output_signs = std::vector<LoopSign>{ LoopSign::NONE };
				rows[i].push_back(stitch);
			}
		}
		
		// Remove output edge from a decrease to a point
		if (to_nothing) {
			rows.back().back().outputs = std::vector<LoopType>{};
			rows.back().back().output_signs = std::vector<LoopSign>{};
		}
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

	std::pair<int, int> Patch::get_segment(int side)
	{
		int i = 0;
		int j = 0;
		while (side > 0) {
			++j;
			while (j >= sides[i % 4].size()) {
				j = 0;
				++i;
			}
			--side;
		}
		return std::make_pair(i%4, j);
	}

	bool Patch::is_loop(int side)
	{
		return get_segment(side).first % 2 == 0;
	}

	bool Patch::is_out(int side)
	{
		auto s = get_segment(side).first;
		return s == 1 || s == 2;
	}

	std::vector<std::shared_ptr<KnitGraphEdge>> Patch::get_edge(int side, bool is_out)
	{
		auto seg = get_segment(side);
		int gen_side = seg.first;
		int sub_side = seg.second;
		int start_idx = 0;
		// Loop Out and Yarn In are ordered in reverse of side order
		if (gen_side % 4 <= 1) {
			for (int i = 0; i < sub_side; ++i) {
				start_idx += sides[gen_side][i];
			}
		}
		else {
			for (int i = sides[gen_side].size() - 1; i > sub_side; --i) {
				start_idx += sides[gen_side][i];
			}
		}
		std::vector<std::shared_ptr<KnitGraphEdge>> edge;

		for (int i = 0; i < sides[gen_side][sub_side]; ++i) {
			edge.push_back(boundaries[gen_side][start_idx + i]);
		}
		return edge;
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
				node->pass_through = stitch.pass_through;
				node->pos = Eigen::RowVector3d(
					(double)(col + 1) / (row.size() + 1),
					(double) (row_num + 1)/(chart.size()+1), 
					 0);
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
		Eigen::MatrixXd V, V3d;
		Eigen::MatrixXi E; // Unused
		graph.edge_list_graph(V3d, E);

		Eigen::MatrixXd C;
		C.resize(corners.rows(), 2);
		V = V3d.block(0, 0, V3d.rows(), 2); // Drop the Z coordinate

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

		int coord = 0;
		Eigen::RowVector2d vtx(0.0, 0.0);
		std::vector<int> duplicated_coords;
		std::vector<Eigen::RowVector2d> corners2d;
		corners2d.push_back(vtx);
		for (int side = 0; side < 4; ++side) {
			std::vector<double> deltas;
			if (side_lengths[side] == 0) {
				// If all segments in a side are size 0, evenly distribute
				for (int l : sides[side]) {
					deltas.push_back((1.0 / sides[side].size()));
				}
			}
			else {
				for (int l : sides[side]) {
					deltas.push_back((double)l / side_lengths[side]);
				}
			}

			// coordinates decrease along top and left edges
			int sign = 1;
			if (side > 1) {
				sign = -1;
			}

			if (sides[side].size() == 0) {
				// If we have no side vertices, duplice the shared corner
				int duplicated = 0;
				for (int s = 0; s < side; ++s) {
					duplicated += sides[s].size();
				}
				duplicated_coords.push_back(duplicated);
				vtx(coord) += sign * 1.0;
				corners2d.push_back(vtx);
			}
			else {
				for (int i = 0; i < deltas.size(); ++i) {
					vtx(coord) += sign * deltas[i];
					if (side < sides.size() - 1 || i < deltas.size() - 1) { // don't duplicate the final corner
						corners2d.push_back(vtx);
					}
				}
			}
			coord = (coord + 1) % 2;
		}
		Eigen::MatrixXd C3d(corners.rows() + duplicated_coords.size(), 3);
		int r = 0;
		int duplicate_index = 0;
		for (int i = 0; i < corners.rows(); ++i) {
			C3d.row(r++) = corners.row(i);
			while (duplicate_index < duplicated_coords.size() && duplicated_coords[duplicate_index] == i) {
				C3d.row(r++) = corners.row(i);
				++duplicate_index;
			}
		}

		C.resize(corners2d.size(), 2);
		for (int i = 0; i < corners2d.size(); ++i) {
			C.row(i) = corners2d[i];
		}

		Eigen::MatrixXd W;
		igl::mvc(V, C, W);
		auto node_positions = W * C3d;

		for (int i = 0; i < W.rows(); ++i) {
			graph.nodes[i]->pos = node_positions.row(i);
		}
	}

	Patch::Patch(std::vector<std::vector<int>> sides, const Eigen::MatrixXd& corners, int shaping, int sr_shaping)
	{
		this->corners = corners;
		this->sides = sides;
		std::vector<int> side_counts;
		for (auto& side : sides) {
			int count = 0;
			for (auto& quad_side : side) {
				count += quad_side;
			}
			side_counts.push_back(count);
		}

		// If we are in a special case (source or sink) build sepate patches then connect

		if (side_counts[0] > 0 && (side_counts[1] == 0 && side_counts[2] == 0 && side_counts[3] == 0)) {
			// Sink case
			std::vector<Patch> patches;
			for (int i = 0; i < sides[0].size(); ++i) {
				int n = sides[0][i];
				std::vector<std::vector<int>> patch_sides{ {n},{n},{0},{n} };
				Eigen::MatrixXd patch_corners(4, 3);
				// TODO - Properly initialize use corners
				patches.emplace_back(patch_sides, patch_corners, shaping, sr_shaping);
			}

			for (int i = 0; i < patches.size(); ++i) {
				for (auto& node : patches[i].graph.nodes) {
					graph.nodes.push_back(node);
				}
				// Only keep forward yarn-edges (don't double count)
				for (auto& edge : patches[i].graph.edges) {
					if (edge->is_loop || edge->src) {
						graph.edges.push_back(edge);
					}
				}
				auto& p1 = patches[i];
				auto& p2 = patches[(i + 1) % patches.size()];
				for (int i = 0; i < p1.boundaries[1].size(); ++i) {
					p1.boundaries[1][i]->dst = p2.boundaries[2][i]->dst;
				}
			}
		}
		else if (side_counts[3] > 0 && (side_counts[1] == 0 && side_counts[2] == 0 && side_counts[0] == 0)) {
			// Sink case
			std::vector<Patch> patches;
			for (int i = 0; i < sides[0].size(); ++i) {
				int n = sides[2][i];
				std::vector<std::vector<int>> patch_sides{ {0},{n},{n},{n} };
				Eigen::MatrixXd patch_corners(4, 3);
				// TODO - Properly initialize use corners
				patches.emplace_back(patch_sides, patch_corners, shaping, sr_shaping);
			}

			for (int i = 0; i < patches.size(); ++i) {
				for (auto& node : patches[i].graph.nodes) {
					graph.nodes.push_back(node);
				}
				// Only keep forward yarn-edges (don't double count)
				for (auto& edge : patches[i].graph.edges) {
					if (edge->is_loop || edge->src) {
						graph.edges.push_back(edge);
					}
				}
				auto& p1 = patches[i];
				auto& p2 = patches[(i + 1) % patches.size()];
				for (int i = 0; i < p1.boundaries[1].size(); ++i) {
					p1.boundaries[1][i]->dst = p2.boundaries[2][i]->dst;
				}
			}
		}
		else {
			// Figure out which case we are in
			assert(side_counts[0] == side_counts[2] || side_counts[1] == side_counts[3]); // only one shaping operation
			Chart c;

			int leaning_dir = 0;
			if (shaping == 1) { leaning_dir = -1; }
			if (shaping == 2) { leaning_dir = 1; }
			if (shaping == 3) { leaning_dir = 0; }

			int sr_dir = 0;
			if (sr_shaping == 1) { sr_dir = -1; }
			if (sr_shaping == 2) { sr_dir = 1; }

			if (side_counts[0] > side_counts[2]) { // Decreases
				
				c.decreases_leaning(side_counts[0], side_counts[2], side_counts[1], leaning_dir);
			}
			else if (side_counts[1] != side_counts[3]) { // Short Rows
				c.short_rows(side_counts[3], side_counts[1], side_counts[0], sr_dir);
			}
			else { // Flat or increases
				c.increases_leaning(side_counts[0], side_counts[2], side_counts[1], leaning_dir);
			}

			make_graph(c.rows);
		}
	}

};