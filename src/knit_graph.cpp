#include "knit_graph.h"
#include "glyph.h"

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
		}

		if (top.size() > 0 && bottom.size() > 0 && top.size() == bottom.size()) {
			for (int i = 0; i < top.size(); ++i) {
				bottom[i]->dst = top[i]->dst; // Keep bottom edge (it will have any tucks, etc. that are needed)
				top[i]->contracted = true;
			}
		}

		contracted = true;
		return true;
	}
	return false;
}
