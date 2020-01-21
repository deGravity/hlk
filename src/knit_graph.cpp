#include "knit_graph.h"
#include "glyph.h"

#include "vkmp/xferplan/Stitch.hpp"
#include "vkmp/scheduler.hpp"

#include "texture.h"

#include <iostream>
#include <set>

#include <deque>

#include "disjointset.h"

#include <igl/parula.h>

//#include <igl/copyleft/cgal/wire_mesh.h>
namespace hlk {
	std::vector<std::shared_ptr<KnitGraph>> hlk::KnitGraph::separate_components()
	{
		re_index();
		IntUnionFind djs(nodes.size());
		for (int i = 0; i < nodes.size(); ++i) {
			std::vector<std::shared_ptr<KnitGraphEdge>> out_edges{ nodes[i]->right };
			out_edges.insert(out_edges.end(), nodes[i]->top.begin(), nodes[i]->top.end());
			for (auto& e : out_edges) {
				if (e->dst) {
					djs.join(i, e->dst->index);
				}
			}
		}

		std::vector<std::shared_ptr<KnitGraph>> subgraphs;

		auto components = djs.components();
		for (auto& component : components) {
			std::shared_ptr<KnitGraph> G = std::make_shared<KnitGraph>();
			for (int i : component) {
				G->nodes.push_back(nodes[i]);
			}
			subgraphs.push_back(G);
		}

		for (auto& G : subgraphs) {
			G->recollect_edges();
		}
			
		return subgraphs;
	}
	void KnitGraph::recollect_edges()
	{
		edges.clear();
		for (auto& node : nodes) {
			if (node->right) {
				edges.push_back(node->right);
			}
			for (auto& up : node -> top) {
				edges.push_back(up);
			}
		}
		re_index();
	}
}

hlk::IGLVisualization hlk::KnitGraph::visualize_stitches(const std::vector<vkmp::Stitch>& stitches)
{
	hlk::GraphVisualization vis;

	vis.V.resize(stitches.size(), 3);
	// Vector pointing to some neighbor at each vertex to compute normals for displaying arrow heads
	Eigen::MatrixXd neighbor_vec(stitches.size(), 3);
	for (int i = 0; i < stitches.size(); ++i) {
		vis.V(i,0) = (double) stitches[i].at.x;
		vis.V(i, 1) = (double) stitches[i].at.y;
		vis.V(i, 2) = (double) stitches[i].at.z;


		for (uint32_t n : {stitches[i].in[0], stitches[i].in[1], stitches[i].out[0], stitches[i].out[1]}) {
			neighbor_vec.row(i) = Eigen::RowVector3d(1, 0, 0);
			if (n != -1U) {
				Eigen::RowVector3d neighbor_pos(stitches[n].at.x, stitches[n].at.y, stitches[n].at.z);
				neighbor_vec.row(i) = neighbor_pos - vis.V.row(i);
				vis.add_line(i, n, Eigen::RowVector3d(1.0, 0.564, 0.0));
			}
		}
	}

	std::map<int, bool> yarn_seen;
	std::map<int, int> last_per_yarn;

	std::set<int> yarn_ids;

	// Figure out how many yarns we have so we can make them all different colors
	for (int i = 0; i < stitches.size(); ++i) {
		yarn_ids.insert(stitches[i].yarn);
	}
	int num_yarns = yarn_ids.size();

	Eigen::MatrixXd yarn_colors(num_yarns, 3);
	if (num_yarns > 1) {
		for (int i = 0; i < num_yarns; ++i) {
			double c, r, g, b;
			c = (double)i / (num_yarns - 1);
			igl::parula(c, r, g, b);
			yarn_colors(i, 0) = r;
			yarn_colors(i, 1) = g;
			yarn_colors(i, 2) = b;
		}
	}
	else {
		yarn_colors.row(0) = Eigen::RowVector3d(0.0, 1.0, 0.0);
	}

	for (int i = 0; i < stitches.size(); ++i) {
		int yarn = stitches[i].yarn;
		if (yarn_seen[yarn]) {
			int prev = last_per_yarn[yarn];
			vis.add_line(prev, i, yarn_colors.row(yarn));
		}

		yarn_seen[yarn] = true;
		last_per_yarn[yarn] = i;
		
		Eigen::RowVector3d point_color = yarn_colors.row(yarn);

		/*
		if (stitches[i].direction == 'a') {
			point_color = Eigen::RowVector3d(0.0, 0.0, 0.0);
		}
		else {
			point_color = Eigen::RowVector3d(1.0, 1.0, 1.0);
		}
		*/
		if (stitches[i].data.id == 1) {
			point_color = Eigen::RowVector3d(1.0, 1.0, 1.0);
		}
		else {//if (stitches[i].data.name == "purl") {
			point_color = Eigen::RowVector3d(0.0, 0.0, 0.0);
		}

		vis.add_point(i, point_color);
		vis.add_label(i, std::to_string(i));
	}

	return vis.get_vis();
}

void hlk::KnitGraph::contract()
{
	// Contract all contractible nodes
	for (auto& node : nodes) {
		node->contract();
	}

	// Delete contracted nodes and edges
	nodes.erase(
		std::remove_if(
			nodes.begin(), nodes.end(),
			[](std::shared_ptr<KnitGraphNode>& n) { return n->contracted; }),
		nodes.end()
	);

	edges.erase(
		std::remove_if(
			edges.begin(), edges.end(),
			[](std::shared_ptr<KnitGraphEdge>& e) { return e->contracted; }),
		edges.end()
	);

	re_index();
}

void hlk::KnitGraph::generate_instructions(std::string base_filename, int depth, bool cse)
{
	/*
	if (!traced) {
		trace();
	}
	*/
	auto components = separate_components();
	int component_num = 0;

	for (auto& component : components) {

		auto filename = base_filename + "_" + std::to_string(component_num);

		component->trace();

		vkmp::Scheduler s;
		s.stitches = component->stitches;

		vkmp::save_stitches(filename + ".st", component->stitches);

		std::map<int, std::pair<int, int>> yarn_mappings;
		yarn_mappings[0] = std::make_pair(1, -1);
		yarn_mappings[1] = std::make_pair(2, -1);
		yarn_mappings[2] = std::make_pair(3, -1);
		s.do_schedule(yarn_mappings, cse, depth);
		s.write_schedule(filename + ".js");
		std::string scripts_dir = SCRIPTS_DIR;
		std::string node_path = "NODE_PATH=" + scripts_dir + "\\";
		putenv(node_path.c_str());
		std::string command_1 = "node " + filename + ".js";
		std::cout << "Trying to run:\n" << command_1 << std::endl;
		system(command_1.c_str());
		std::string command_2 = "node " + scripts_dir + "\\knitout-to-dat.js " + filename + ".k " + filename + ".dat";
		std::cout << "Trying to run:\n" << command_2 << std::endl;
		system(command_2.c_str());

		++component_num;
	}
	re_index();
}

void hlk::KnitGraph::propogate_textures()
{
	std::vector<int> texture_r(nodes.size(), -1);
	std::vector<int> texture_c(nodes.size(), -1);
	std::vector<bool> explored(nodes.size(), false);

	// BFS to Propogate Texture Coordinates
	for (int n = 0; n < nodes.size(); ++n) {
		if (explored[n]) continue; // Already Explored

		texture_r[n] = 0;
		texture_c[n] = 0;
		explored[n] = true;

		std::deque<int> to_explore;

		to_explore.push_back(n);

		auto explore_neighbor = [&](int curr, int neigh, int axis, int step, int other_step = 0) {
			if (!explored[neigh] && nodes[curr]->texture_id == nodes[neigh]->texture_id) {
				if (axis == 0) {
					texture_r[neigh] = texture_r[curr + other_step];
					texture_c[neigh] = texture_c[curr] + step;
				}
				else {
					texture_r[neigh] = texture_r[curr] + step;
					texture_c[neigh] = texture_c[curr + other_step];
				}
				explored[neigh] = true;
				to_explore.push_back(neigh);
			}
		};

		while (!to_explore.empty()) {
			int current = to_explore.front();
			to_explore.pop_front();
			if (nodes[current]->right) {
                if (nodes[current]->right->dst) {
                    int neighbor = nodes[current]->right->dst->index;
                    explore_neighbor(current, neighbor, 0, 1);
                }
			}
			if (nodes[current]->left) {
                if (nodes[current]->left->src) {
                    int neighbor = nodes[current]->left->src->index;
                    explore_neighbor(current, neighbor, 0, -1);
                }
			}
			int i = 0;
			for (auto& child : nodes[current]->top) {
                if (child->dst) {
                    int neighbor = child->dst->index;
                    explore_neighbor(current, neighbor, 1, 2, i);
					++i;
                }
			}
			i = 0;
			for (auto& parent : nodes[current]->bottom) {
                if (parent->src) {
                    int neighbor = parent->src->index;
                    explore_neighbor(current, neighbor, 1, -2, i);
					++i;
                }
			}
		}

	}

	auto texture_db = get_textures();

	// Now Apply the Textures
	for (int n = 0; n < nodes.size(); ++n) {
		auto& texture = texture_db[nodes[n]->texture_id];
		int R = texture.pattern.rows();
		int C = texture.pattern.cols();
		int r_i = (R + (texture_r[n] % R)) % R;
		int r_l = (R + ((texture_r[n] + 1) % R)) % R;
		int c = (C + (texture_c[n] % C)) % C;
		nodes[n]->internal_knit = texture.pattern(r_i, c) == 0;
		for (auto& top : nodes[n]->top) {
			if (top->is_loop && top->type == LoopType::KNIT) {
				top->type = (texture.pattern(r_l, c) == 0) ? LoopType::KNIT : LoopType::PURL;
			}
		}
	}
}

void hlk::KnitGraph::load_st(std::string filename)
{
	vkmp::load_stitches(filename, &stitches);
	traced = true;
}

void hlk::KnitGraph::trace()
{
	ak::RowColGraph RCG = make_row_col_graph();
	std::vector<ak::TracedStitch> traced_stitches;
	ak::trace_graph(RCG, &traced_stitches);

	// HACK! - Connect childless nodes to their neighbors' children when possible.
	// This fixes special faces


	stitches.reserve(traced_stitches.size());
	std::vector<bool> first_trace(nodes.size(), true);
	for (auto const& ts : traced_stitches) {
		stitches.emplace_back();
		stitches.back().yarn = ts.yarn;
		stitches.back().type = ts.type;
		stitches.back().direction = ts.dir;
		stitches.back().in[0] = ts.ins[0];
		stitches.back().in[1] = ts.ins[1];
		stitches.back().out[0] = ts.outs[0];
		stitches.back().out[1] = ts.outs[1];
		stitches.back().at = ts.at;

		stitches.back().data = nodes[ts.vertex]->get_data(first_trace[ts.vertex]);

		first_trace[ts.vertex] = false;

	}

	// Now go back and put in yarn-ends at the first and last instances of
	// each yarn
	typedef struct {
		uint32_t first = -1U;
		uint32_t last = -1U;
	} yarn_endpoints;
	std::map<int, yarn_endpoints> yarn_ends;
	std::set<int> yarn_ids;
	for (int i = 0; i < traced_stitches.size(); ++i) {
		int yarn = traced_stitches[i].yarn;
		if (yarn_ends[yarn].first == -1U) {
			yarn_ends[yarn].first = i;
		}
		yarn_ends[yarn].last = i;
		yarn_ids.insert(yarn);
	}
	for (int yarn : yarn_ids) {
		if (stitches[yarn_ends[yarn].first].in[0] == -1U) {
			stitches[yarn_ends[yarn].first].data = STITCH::YARNEND;
		}
		if (stitches[yarn_ends[yarn].last].out[0] == -1U) {
			stitches[yarn_ends[yarn].last].data = STITCH::YARNEND;
		}
	}

	traced = true;
}

ak::RowColGraph hlk::KnitGraph::make_row_col_graph()
{
	ak::RowColGraph G;

	G.vertices.resize(nodes.size());

	for (int i = 0; i < nodes.size(); ++i) {
		G.vertices[i].at = nodes[i]->pos;
		if (nodes[i]->left && nodes[i]->left->src) {
			G.vertices[i].row_in = nodes[i]->left->src->index;
		}
		if (nodes[i]->right && nodes[i]->right->dst) {
			G.vertices[i].row_out = nodes[i]->right->dst->index;
		}
		assert(nodes[i]->bottom.size() <= 2);
		for (auto& parent : nodes[i]->bottom) {
			if (parent->src) {
				G.vertices[i].add_col_in(parent->src->index);
			}
		}
		assert(nodes[i]->top.size() <= 2);
		for (auto& child : nodes[i]->top) {
			if (child->dst) {
				G.vertices[i].add_col_out(child->dst->index);
			}
		}
	}

	return G;
}

void hlk::KnitGraph::re_index()
{
	for (int i = 0; i < nodes.size(); ++i) {
		nodes[i]->index = i;
	}
	for (int i = 0; i < edges.size(); ++i) {
		edges[i]->index = i;
	}
}

void hlk::KnitGraph::edge_list_graph(Eigen::MatrixXd & V, Eigen::MatrixXi & E)
{
	V.resize(nodes.size(), 3);
	E.resize(0, 2);

	re_index();

	for (int i = 0; i < nodes.size(); ++i) {
		V.row(i) = nodes[i]->pos;
	}
	for (int i = 0; i < edges.size(); ++i) {
		// Ignore boundary edges
		if (edges[i]->src && edges[i]->dst) {
			E.conservativeResize(E.rows() + 1, 2);
			E(E.rows() - 1, 0) = edges[i]->src->index;
			E(E.rows() - 1, 1) = edges[i]->dst->index;
		}
	}
}

void hlk::KnitGraph::build_mesh(double th, int poly_size, Eigen::MatrixXd & V, Eigen::MatrixXi & F, Eigen::MatrixXd & C)
{
	// Since I can't get wire_mesh to work statically, use points API for now
	// Caller will need to choose and set a color for points still

	Eigen::MatrixXd WV;
	Eigen::MatrixXi WE;
	//Eigen::Matrix<int, -1, -1, 0, -1, 1> J;

	edge_list_graph(WV, WE);
	//igl::copyleft::cgal::wire_mesh(WV, WE, th, poly_size, V, F, J);

	V = WV;
	F = WE; // F is really edges

	C.resize(F.rows(), 3);

	int c = 0;
	for (int i = 0; i < edges.size(); ++i) {
		if (edges[i]->dst && edges[i]->src) {
			if (edges[i]->is_loop) {
				C.row(c++) = color::ORANGE.block(0,0,1,3);
			}
			else {
				C.row(c++) = color::GREEN.block(0,0,1,3);
			}
		}
	}

	/*
	for (int j = 0; j < F.rows(); ++j) {
		if (J(j) < WV.rows()) { // Vertex Face
			int v = j;
			C.row(j) = Eigen::RowVector3d(1.0, 1.0, 1.0);
		}
		else { // Edge Face
			int e = j - WV.rows();
			if (edges[e]->is_loop) {
				C.row(j) = Eigen::RowVector3d(1.0, 0.564, 0.0); // Orange
			}
			else {
				C.row(j) = Eigen::RowVector3d(0.149, 1.0, 0.149); // Green
			}
		}
	}
	*/
}

bool hlk::KnitGraphNode::contractable()
{
	// No pass through for increases / decreases
	if (pass_through && top.size() < 2 && bottom.size() < 2) {
		return true;
	}

	// contractable if no interacting yarns
	bool can_contract = (top.size() == 0 && bottom.size() == 0) || (!left && !right);
	

	if (can_contract) {
		// Only contract if no dangling edges (don't want to contract things we may need later
		std::vector<std::shared_ptr<KnitGraphEdge>> edges;
		for (auto& e : top) {
			edges.push_back(e);
		}
		for (auto& e : bottom) {
			edges.push_back(e);
		}
		if (left) {
			edges.push_back(left);
		}
		if (right) {
			edges.push_back(right);
		}
		bool do_contract = true;
		for (auto& edge : edges) {
			if (!edge->src || !edge->dst) {
				do_contract = false;
			}
		}
		return do_contract;
	}
	return false;
}

bool hlk::KnitGraphNode::contract()
{
	if (contractable()) {
		if (left && right) {
			left->dst = right->dst;
			right->contracted = true;
			// Also need to update pointers
			if (left->dst) { // If right was a dangling edge, no pointer to update
				left->dst->left = left;
			}
		}

		if (top.size() > 0 && bottom.size() > 0 && top.size() == bottom.size()) {
			for (int i = 0; i < top.size(); ++i) {
				bottom[i]->dst = top[i]->dst; // Keep bottom edge (it will have any tucks, etc. that are needed)
				top[i]->contracted = true;

				// Also need to update pointers
				if (bottom[i]->dst) {
					// Possible locatons of the pointer we need to update
					auto& candidates = bottom[i]->dst->bottom;
					for (int j = 0; j < candidates.size(); ++j) {
						if (candidates[j] == top[i]) {
							candidates[j] = bottom[i];
						}
					}
				}

			}
		}

		contracted = true;
		return true;
	}
	return false;
}

vkmp::StData hlk::KnitGraphNode::get_data(bool first_tracing)
{
	vkmp::StData data;

	data = first_tracing ? (internal_knit ? STITCH::KNIT : STITCH::PURL) : STITCH::KNIT;

	// Knit and Purl
	if (top.size() == 1 && bottom.size() == 1 && left && right) {
		if (!first_tracing) {
			auto type = top[0]->type;
			switch (type) {
			case LoopType::KNIT:
				data = STITCH::KNIT;
				break;
			case LoopType::PURL:
				data = STITCH::PURL;
				break;
			case LoopType::SLIP:
				data = STITCH::MISS;
				break;
			case LoopType::YARNOVER:
				// This would be a 2-pass: first drop, then tuck
				// Doesn't make a lot of sense
				break;
			}
		}
	}
	// Increases
	if (top.size() > bottom.size()) {
		data = internal_knit ? STITCH::KNIT : STITCH::PURL;
		if (top.size() == 2 && bottom.size() == 1 && left && right) {
			// TODO - How to make hidden (split) increases?
			data = STITCH::INCREASE;
			if (top[1]->type == LoopType::YARNOVER) {
				data = STITCH::INCREASE_R;
			}
			else if (top[0]->type == LoopType::YARNOVER) {
				data = STITCH::INCREASE_L;
			}
		}
		// Don't output the increase on the first tracing
		if (first_tracing) {
			data = internal_knit ? STITCH::KNIT : STITCH::PURL;
		}

	}
	// Decreases
	if (bottom.size() == 2 && top.size() == 1 && left && right) {
		data = STITCH::BINDOFF;
		if (top.size() == 1 && bottom.size() == 2 && left && right) {
			data = STITCH::DECREASE;
			
			/*
			if (loop_stacking[0] == 0) {
				data = STITCH::DEC_R;
			}
			else {
				data = STITCH::DEC_L;
			}
			*/
			
		}
		// Don't output the increase on the second tracing
		if (!first_tracing) {
			data = top[0]->type == LoopType::PURL ? STITCH::PURL : STITCH::KNIT;
		}
	}

	return data;
}
