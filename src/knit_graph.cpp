#include "knit_graph.h"
#include "glyph.h"

#include "vkmp/xferplan/Stitch.hpp"
#include "vkmp/scheduler.hpp"

#include <iostream>
//#include <igl/copyleft/cgal/wire_mesh.h>

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

void hlk::KnitGraph::trace(std::string filename)
{
	ak::RowColGraph RCG = make_row_col_graph();
	std::vector<ak::TracedStitch> traced_stitches;
	ak::trace_graph(RCG, &traced_stitches);


	vkmp::Scheduler s;
	std::vector< vkmp::Stitch >& stitches = s.stitches;
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

		// First tracing gets a
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
		stitches[yarn_ends[yarn].first].data = STITCH::YARNEND;
		stitches[yarn_ends[yarn].last].data = STITCH::YARNEND;
	}
	
	//vkmp::load_stitches(filename, &s.stitches);
	
	vkmp::save_stitches(filename + ".st", stitches);

	std::map<int, std::pair<int, int>> yarn_mappings;
	yarn_mappings[0] = std::make_pair(1, -1);
	yarn_mappings[1] = std::make_pair(2, -1);
	s.do_schedule(yarn_mappings, true, - 1);
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

	
	//ak::save_traced(filename, traced_stitches);
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

	data = STITCH::KNIT; // Default to a knit stitch
	// Knit and Purl
	if (top.size() == 1 && bottom.size() == 1 && left && right) {
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
	// Increases
	if (top.size() > bottom.size()) {
		data = STITCH::CASTON;
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
		// TODO - Should choose appropriately between knit and purl
		if (first_tracing) {
			data = STITCH::KNIT;
		}
	}
	// Decreases
	if (bottom.size() == 2 && top.size() == 1 && left && right) {
		data = STITCH::BINDOFF;
		if (top.size() == 1 && bottom.size() == 2 && left && right) {
			data = STITCH::DECREASE;
			if (loop_stacking[0] == 0) {
				data = STITCH::DEC_R;
			}
			else {
				data = STITCH::DEC_L;
			}
		}
		// Don't output the increase on the second tracing
		// TODO - Should choose appropriately between knit and purl
		if (!first_tracing) {
			data = STITCH::KNIT;
		}
	}

	return data;
}
