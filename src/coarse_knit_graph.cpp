#include "coarse_knit_graph.h"

namespace hlk {
	CoarseKnitGraph::PatchData::PatchData(
		std::vector<std::vector<int>> side_lengths, 
		const Eigen::MatrixXd& corners,
		int t) 
	{
		this->corners = corners;
		this->side_lengths = side_lengths;
		this->time = t;
	}
	void CoarseKnitGraph::visualize(
		Eigen::MatrixXd& P, 
		Eigen::MatrixXd& P_c, 
		Eigen::MatrixXd& V, 
		Eigen::MatrixXi& E, 
		Eigen::MatrixXd& E_c, 
		std::vector<std::string>& L, 
		Eigen::MatrixXd& L_p)
	{

		// Make sure sizes are setup correctly and cleared
		// TODO - If we expect non-empty containers to start,
		// don't clear them. We could use a struct w/ proper initialization
		// or just assert that they have the correct dims to start
		P.resize(0, 3);
		P_c.resize(0, 3);
		V.resize(0, 3);
		E.resize(0, 2);
		E_c.resize(0, 3);
		L.clear();
		L_p.resize(0, 3);

		auto add_point = [&](const Eigen::RowVector3d& pos, const Eigen::RowVector3d& color) {
			P.conservativeResize(P.rows() + 1, P.cols());
			P_c.conservativeResize(P_c.rows() + 1, P_c.cols());
			P.row(P.rows() - 1) = pos;
			P_c.row(P_c.rows() - 1) = color;
		};
		
		auto add_line = [&](const Eigen::RowVector3d& a, const Eigen::RowVector3d& b, const Eigen::RowVector3d& color) {
			V.conservativeResize(V.rows() + 2, V.cols());
			V.row(V.rows() - 2) = a;
			V.row(V.rows() - 1) = b;
			E.conservativeResize(E.rows() + 1, E.cols());
			E.row(E.rows() - 1) = Eigen::RowVector2i(V.rows() - 2, V.rows() - 1);
			E_c.conservativeResize(E_c.rows() + 1, E_c.cols());
			E_c.row(E_c.rows() - 1) = color;
		};

		auto add_arrow = [&](
			const Eigen::RowVector3d& a, 
			const Eigen::RowVector3d& b, 
			const Eigen::RowVector3d& N, 
			const Eigen::RowVector3d& color,
			double pct) {
				double len = (b - a).norm();
				Eigen::RowVector3d dir = (b - a).normalized();
				Eigen::RowVector3d A = a + (1.0 - pct) / 2 * dir;
				Eigen::RowVector3d B = b - (1.0 - pct) / 2 * dir;
				Eigen::RowVector3d X = dir.cross(N).normalized();
				Eigen::RowVector3d tip1 = B + X * len * 0.1 - dir * len * 0.1;
				Eigen::RowVector3d tip2 = B - X * len * 0.1 - dir * len * 0.1;
				add_line(A, B, color);
				add_line(B, tip1, color);
				add_line(B, tip2, color);
		};

		auto add_label = [&](const Eigen::RowVector3d& pos, const std::string& text) {
			L.push_back(text);
			L_p.conservativeResize(L_p.rows() + 1, L_p.cols());
			L_p.row(L_p.rows() - 1) = pos;
		};

		Eigen::MatrixXd patch_centers;
		patch_centers.resize(patch_data.size(), 3);
		Eigen::MatrixXd patch_normals = Eigen::MatrixXd::Zero(patch_data.size(), 3);

		for (int p = 0; p < patch_data.size(); ++p) {			
			auto& patch = patch_data[p];
			int s = 0;
			int sub_s = 0;
			for (int i = 0; i < patch.corners.rows(); ++i) {
				int j = (i + 1) % patch.corners.rows();
				int k = (j + 1) % patch.corners.rows();
				Eigen::RowVector3d A = patch.corners.row(i);
				Eigen::RowVector3d B = patch.corners.row(j);
				Eigen::RowVector3d C = patch.corners.row(k);

				Eigen::RowVector3d N = (C - B).cross(B - A).normalized();
				patch_normals.row(p) = patch_normals.row(p) + N / patch.corners.rows();

				// Add Grey Outline of patch
				add_line(A, B, Eigen::RowVector3d(.65, .65, .65));
				while (sub_s >= patch.side_lengths[s % patch.side_lengths.size()].size()) {
					sub_s = 0;
					++s;
				}

				// Add a side and stitch-count label
				//std::string side_label = "(" + std::to_string(p) + "," + std::to_string(i) + ") = " + std::to_string(patch.side_lengths[s][sub_s]);
				//add_label(A / 3 + 2 * B / 3, side_label);
				add_label((A + B) / 2, std::to_string(patch.side_lengths[s][sub_s]));
				++sub_s;
			}
			// Add a patch-center vertex
			Eigen::RowVector3d center(0.0, 0.0, 0.0);
			center = patch.corners.colwise().mean();
			patch_centers.row(p) = center;

			std::string patch_label = std::to_string(p) + "@" + std::to_string(patch.time);

			add_point(center, Eigen::RowVector3d(.4, .4, .4));
			//add_label(center, patch_label);
		}

		for (int e = 0; e < edges.size(); ++e) {
			auto& edge = edges[e];
			Eigen::RowVector3d src_p = patch_centers.row(edge.src);
			Eigen::RowVector3d dst_p = patch_centers.row(edge.dst);
			Eigen::RowVector3d src_n = patch_normals.row(edge.src);
			Eigen::RowVector3d dst_n = patch_normals.row(edge.dst);
			Eigen::RowVector3d color(1.0, 0.564, 0.0);
			if (!edge.is_loop) {
				color = Eigen::RowVector3d(0.149, 1.0, 0.149);
			}
			add_arrow(src_p, dst_p, dst_n, color, .95);
		}
	}
	void CoarseKnitGraph::initialize_patches()
	{
		patches.clear();
		patches.reserve(patch_data.size());
		for (int i = 0; i < patch_data.size(); ++i) {
			auto& data = patch_data[i];
			patches.emplace_back(data.side_lengths, data.corners, data.shaping, data.short_row_shaping);
			for (auto& node : patches.back().graph.nodes) {
				node->texture_id = data.texture_id;
			}
		}
		patches_initialized = true;
	}
	KnitGraph CoarseKnitGraph::build_graph()
	{
		if (!patches_initialized) {
			initialize_patches();
		}
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

				// Update the node's edge-pointer
				// We only need to update the pointer on the dst side
				if (src_edges[i]->is_loop) {
					// There should only every be one loop in due to simple patch borders
					// but searching for the right one to replace is more flexible
					auto& candidates = src_edges[i]->dst->bottom;
					for (int j = 0; j < candidates.size(); ++j) {
						if (candidates[j] == dst_edges[i]) {
							candidates[j] = src_edges[i];
						}
					}
				}
				else {
					src_edges[i]->dst->left = src_edges[i];
				}
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

		G.contract();
		G.re_index();

		
		// Find Boundary loops and add cast-on / bind-off stitches

		// First find all the dangling edges
		std::vector<int> starts;
		std::vector<int> ends;
		std::vector<bool> is_start(G.nodes.size(), false);
		std::vector<bool> is_end(G.nodes.size(), false);
		for (auto& e : G.edges) {
			if (e->is_loop) {
				if (!e->dst && e->src) {
					ends.push_back(e->src->index);
					is_end[e->src->index] = true;
				}
				if (!e->src && e->dst) {
					starts.push_back(e->dst->index);
					is_start[e->dst->index] = true;
				}
			}
		}
		std::vector<bool> used(G.nodes.size(), false);

		std::vector<std::vector<int>> start_loops;
		std::vector<std::vector<int>> end_loops;
		auto& nodes = G.nodes;


		// Start Loops
		for (int n : starts) {
			if (!used[n]) {
				std::vector<int> loop;
				used[n] = true;				
				loop.push_back(n);
				int current = n;
				while (true) {
					int candidate = current;
					do {
						auto& cn = nodes[candidate];
						/*if (current != n && cn->left && cn->left->src && !used[cn->left->src->index]) {
							candidate = cn->left->src->index;
						}
						else*/ if (cn->bottom.size() > 0 && cn->bottom[0]->src && (!used[cn->bottom.front()->src->index] || cn->bottom.front()->src->index == n)) {
							candidate = cn->bottom.front()->src->index;
						}
						else if (cn->right && cn->right->dst && (!used[cn->right->dst->index] || cn->right->dst->index == n)) {
							candidate = cn->right->dst->index;
						}
						else if (cn->top.size() > 0 && cn->top.back()->dst && !used[cn->top.back()->dst->index]) {
							candidate = cn->top.back()->dst->index;
						}
						else {
							break; // Stop if we can't find a direction to go
						}
					} while (!is_start[candidate]);
					if (candidate != n) {
						current = candidate;
						loop.push_back(current);
						used[current] = true;
					}
					else {
						break;
					}
				}
				start_loops.push_back(loop);
			}
		}

		// End Loops
		for (int n : ends) {
			if (!used[n]) {
				std::vector<int> loop;
				used[n] = true;
				loop.push_back(n);
				int current = n;
				while (true) {
					int candidate = current;
					do {
						auto& cn = nodes[candidate];
						/*if (current != n && cn->left && cn->left->src && !used[cn->left->src->index]) {
							candidate = cn->left->src->index;
						}
						else */if (cn->top.size() > 0 && cn->top.back()->dst && !used[cn->top.back()->dst->index]) {
							candidate = cn->top.back()->dst->index;
						}
						else if (cn->right && cn->right->dst && (!used[cn->right->dst->index] || cn->right->dst->index == n)) {
							candidate = cn->right->dst->index;
						}
						else if (cn->bottom.size() > 0 && cn->bottom.front()->src && (!used[cn->bottom.front()->src->index] || cn->bottom.front()->src->index == n)) {
							candidate = cn->bottom.front()->src->index;
						}
						else {
							break; // Stop if we can't find a direction to go
						}
					} while (!is_end[candidate]);
					if (candidate != n) {
						current = candidate;
						loop.push_back(current);
						used[current] = true;
					}
					else {
						break;
					}
				}
				end_loops.push_back(loop);
			}
		}

		// Now Bind-Off / Cast-On on the boundary loops we found

		
		// First the cast-ons
		for (auto& loop : start_loops) {
			std::vector<std::shared_ptr<KnitGraphNode>> loop_nodes;
			for (int k = 0; k < loop.size(); ++k) {
				int i = loop[k];
				G.nodes.push_back(std::make_shared<KnitGraphNode>());
				auto& node = G.nodes.back();
				loop_nodes.push_back(node);
				Eigen::RowVector3d delta(0, 0, 0);
				for (auto& neighbor : nodes[i]->top) {
					delta = delta + (nodes[i]->pos - neighbor->dst->pos) / nodes[i]->top.size();
				}
				node->pos = nodes[i]->pos + delta;

				if (!nodes[i]->bottom.size() > 0) {
					nodes[i]->bottom.push_back(std::make_shared<KnitGraphEdge>());
					G.edges.push_back(nodes[i]->bottom[0]);
					nodes[i]->bottom[0]->dst = nodes[i];
					nodes[i]->bottom[0]->is_loop = true;
				}
				nodes[i]->bottom[0]->src = node;
				nodes[i]->bottom[0]->dst = nodes[i];
				node->top.push_back(nodes[i]->bottom[0]);
				node->patch_id = nodes[i]->patch_id;
			}
			for (int i = 0; i < loop.size(); ++i) {
				int j = (i + 1) % loop_nodes.size();
				G.edges.push_back(std::make_shared<KnitGraphEdge>());
				loop_nodes[i]->right = G.edges.back();
				loop_nodes[j]->left = G.edges.back();
				G.edges.back()->src = loop_nodes[i];
				G.edges.back()->dst = loop_nodes[j];
				G.edges.back()->is_loop = false;
			}
		}

		// Now the Bind-Offs
		for (auto& loop : end_loops) {
			std::vector<std::shared_ptr<KnitGraphNode>> loop_nodes;
			for (int k = 0; k < loop.size(); ++k) {
				int i = loop[k];
				G.nodes.push_back(std::make_shared<KnitGraphNode>());
				auto& node = G.nodes.back();
				loop_nodes.push_back(node);
				Eigen::RowVector3d delta(0, 0, 0);
				for (auto& neighbor : nodes[i]->bottom) {
					delta = delta + (nodes[i]->pos - neighbor->src->pos) / nodes[i]->bottom.size();
				}
				node->pos = nodes[i]->pos + delta;

				if (!nodes[i]->top.size() > 0) {
					nodes[i]->top.push_back(std::make_shared<KnitGraphEdge>());
					G.edges.push_back(nodes[i]->top[0]);
					nodes[i]->top[0]->src = nodes[i];
					nodes[i]->top[0]->is_loop = true;
				}
				nodes[i]->top[0]->dst = node;
				nodes[i]->top[0]->src = nodes[i];
				node->bottom.push_back(nodes[i]->top[0]);
				node->patch_id = nodes[i]->patch_id;
			}
			for (int i = 0; i < loop.size(); ++i) {
				int j = (i + 1) % loop_nodes.size();
				G.edges.push_back(std::make_shared<KnitGraphEdge>());
				loop_nodes[i]->right = G.edges.back();
				loop_nodes[j]->left = G.edges.back();
				G.edges.back()->src = loop_nodes[i];
				G.edges.back()->dst = loop_nodes[j];
				G.edges.back()->is_loop = false;
			}
		}
		
		G.re_index();

		G.propogate_textures();
		
		return G;
	}
};