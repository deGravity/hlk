#include "coarse_knit_graph.h"

namespace hlk {
	CoarseKnitGraph::CoarseKnitGraph(const CoarseKnitMesh& mesh)
	{
		nodes.resize(mesh.quads.size());
		for (int i = 0; i < nodes.size(); ++i) {
			auto& quad = mesh.quads[i];
			auto& node = nodes[i];
			int q = quad.index; // Sholud be i, but just incase this ever changes

			quad.get_corners(node.corners);
			
			std::vector<int> split_points;
			std::vector<int> split_sides;

			for (int j = 0; j < 4; ++j) {
				int s = quad.index + j;
				node.side_stitches.push_back(mesh.sides[s].stitches->val);
			}

			auto idx = [](bool is_loop, bool is_out)->int {
				return ((is_loop ? 0 : 3) + (is_out ? 2 : 0)) % 4;
			};

			for (int j = 0; j < 4; ++j) {
				int k = (j + 3) % 4; // Previous side
				auto& side_a = mesh.sides[q + k];
				auto& side_b = mesh.sides[q + j];
				if (side_a.is_loop->val != side_b.is_loop->val || side_a.is_out->val != side_b.is_out->val) {
					split_points.push_back(j);
					split_sides.push_back(idx(side_b.is_loop->val, side_b.is_out->val));
				}
			}

			node.sides.resize(4);
			for (int j = 0; j < split_points.size(); ++j) {
				int next = split_points[(j + 1) % split_points.size()];
				int val = split_sides[j];
				int k = j;
				while (k < next) {
					node.sides[val].push_back(k);
					++k;
				}
			}

			node.init_patch();
		}
	}
	void CoarseKnitGraph::Node::init_patch()
	{
		patch = std::make_shared<Patch>(sides, corners);
	}
};