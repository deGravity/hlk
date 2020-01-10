#pragma once

#include <vector>
#include <Eigen/Core>
#include <string>
#include <map>

#include "knit_graph.h"

namespace hlk {

	struct ChartCell {
		int num_inputs;
		std::vector<int> input_order;
		std::vector<LoopType> outputs;
		std::vector<LoopSign> output_signs;
		bool has_yarn_in;
		bool has_yarn_out;
		bool pass_through = false;
	};


	struct Chart {
		std::vector<std::vector<ChartCell>> rows;

		ChartCell parse_stitch(const std::string& stitch);
		void read_from_file(std::string path);

		std::map<std::string, ChartCell> stitch_db;
		void setup_stitch_db();

		std::vector<int> interpolate(int first, int last, int rows);
		std::vector<int> row_diffs(const std::vector<int>& rows);

		void decreases_leaning(int bottom, int top, int height, int dir);
		void increases_leaning(int bottom, int top, int height, int dir);
		void short_rows(int left, int right, int width, int dir);
	};

	
	struct Patch {

		KnitGraph graph;
		std::vector<std::vector<std::shared_ptr<KnitGraphEdge>>> boundaries;
		std::vector<std::vector<int>> sides;
		Eigen::MatrixXd corners;

		std::pair<int, int> get_segment(int side);
		bool is_loop(int side);
		bool is_out(int side);
		std::vector<std::shared_ptr<KnitGraphEdge>> get_edge(int side, bool is_out);

		void make_graph(std::vector<std::vector<ChartCell>> chart);
		void interpolate_coordinates();

		Patch(std::vector<std::vector<int>> sides, const Eigen::MatrixXd& corners);
	};
};