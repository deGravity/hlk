#include "coarse_knit_graph.h"

namespace hlk {
	KnitGraph CoarseKnitGraph::build_graph()
	{
		KnitGraph G;
		for (auto& patch : patches) {
			patch.interpolate_coordinates();
		}
		for (auto& edge : edges) {
			// Connect src and dst
			auto& p_src = patches[edge.src];
			auto& p_dst = patches[edge.dst];

			auto src_edges = p_src.get_edge(edge.src_side, true);
			auto dst_edges = p_dst.get_edge(edge.dst_side, false);

			assert(src_edges.size() == dst_edges.size()); // edge sizes should align

			for (int i = 0; i < src_edges.size(); ++i) {
				src_edges[i]->dst = dst_edges[i]->dst;
				dst_edges[i]->contracted = true; // Only keep src edge
			}
		}

		for (auto& patch : patches) {
			for (auto& n : patch.graph.nodes) {
				if (!n->contracted) {
					G.nodes.push_back(n);
				}
			}
			for (auto& e : patch.graph.edges) {
				if (!e->contracted) {
					G.edges.push_back(e);
				}
			}
		}

		return G;
	}
};