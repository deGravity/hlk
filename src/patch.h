#pragma once

#include <vector>
#include <Eigen/Core>

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
	
	struct Patch {

		KnitGraph graph;
		std::vector<std::vector<KnitGraphEdge>> boundaries;
		std::vector<std::vector<int>> sides;
		Eigen::MatrixXd corners;

		void make_graph(std::vector<std::vector<ChartCell>> chart);

		Patch(std::vector<std::vector<int>> sides, Eigen::MatrixXd corners);
	};
};