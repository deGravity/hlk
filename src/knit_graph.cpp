#include "knit_graph.h"

#include <iostream>
//#include <igl/copyleft/cgal/wire_mesh.h>

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
				C.row(c++) = Eigen::RowVector3d(1.0, 0.564, 0.0);
			}
			else {
				C.row(c++) = Eigen::RowVector3d(0.149, 1.0, 0.149);
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
