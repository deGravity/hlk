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
	};


	struct Chart {
		std::vector<std::vector<ChartCell>> rows;

		ChartCell parse_stitch(const std::string& stitch);
		void read_from_file(std::string path);

		std::map<std::string, ChartCell> stitch_db;
		void setup_stitch_db();
	};

	
	struct Patch {

		KnitGraph graph;
		std::vector<std::vector<std::shared_ptr<KnitGraphEdge>>> boundaries;
		std::vector<std::vector<int>> sides;
		Eigen::MatrixXd corners;

		void make_graph(std::vector<std::vector<ChartCell>> chart);
		void interpolate_coordinates();

		Patch() {}

		Patch(std::vector<std::vector<int>> sides, Eigen::MatrixXd corners);
	};
};