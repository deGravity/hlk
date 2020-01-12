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

		struct PatchData {
			PatchData(std::vector<std::vector<int>> side_lengths, const Eigen::MatrixXd& corners, int time);
			Eigen::MatrixXd corners;
			std::vector<std::vector<int>> side_lengths;
			int time;
			int texture_id;
			int shaping;
			int short_row_shaping;
		};

		void visualize(
			Eigen::MatrixXd& P,
			Eigen::MatrixXd& P_c,
			Eigen::MatrixXd& V,
			Eigen::MatrixXi& E,
			Eigen::MatrixXd& E_c,
			std::vector<std::string>& L,
			Eigen::MatrixXd& L_p
		);

		std::vector<PatchData> patch_data;
		std::vector<Patch> patches;
		std::vector<Edge> edges;
		bool patches_initialized = false;
		void initialize_patches();

		KnitGraph build_graph();

	};

	
};