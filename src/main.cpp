#define IGL_VIEWER_VIEWER_QUIET 1
// This does nothing during static compilation - will need to add it to the
// CMAKE for it to stick
#include <iostream>

#include "labeling_ui.h"
#include "remeshing_plugin.h"
#include "patch.h"

#include "autoknit.h"
#include "coarse_knit_graph.h"
#include "vkmp/scheduler.hpp"

using namespace hlk;
using namespace std;

int main(void) {

	int mode = 2;

	/*
	cout << "Choose an Interface" << endl;
	cout << "1) Remeshing" << endl;
	cout << "2) Labeling" << endl;
	cin >> mode;
	*/

	if (mode == 1) {
		igl::opengl::glfw::Viewer viewer;
		int rosy = 4;
		std::string input_path = "./tmp_in.obj";
		std::string output_path = "./tmp_out.obj";
		std::string input_model = "";
		RemeshingMenu remeshing_menu(rosy, input_path, output_path);
		if (!input_model.empty()) {
			remeshing_menu.set_input_model(input_model);
		}
		viewer.plugins.push_back(&remeshing_menu);
		viewer.launch();
	}
	else if (mode ==2) {
		igl::opengl::glfw::Viewer viewer2;
		LabelingUI labelingui;
		viewer2.plugins.push_back(&labelingui);
		viewer2.launch();
	}
	else if (mode == 3) { // test patch visualization
		//std::string filename = igl::file_dialog_open();
		std::vector<std::vector<int>> sides{ {7},{14},{7},{9} };
		Eigen::MatrixXd corners(4, 3);
		corners <<
			0, 0, 0,
			1, 0, 0,
			1, 1, 0,
			0, 1, 0;
		Patch p(sides, corners, 1, 1);

		Eigen::MatrixXd V;
		Eigen::MatrixXi E;
		Eigen::MatrixXd C;
		


		p.interpolate_coordinates();
		p.graph.build_mesh(0.1, 4, V, E, C);

		std::cout << V << std::endl << std::endl << E << std::endl;

		igl::opengl::glfw::Viewer viewer;
		viewer.data().set_points(V, Eigen::RowVector3d(1.0, 1.0, 1.0));
		for (int i = 0; i < V.rows(); ++i) {
			viewer.data().add_label(V.row(i), std::to_string(i) + "\n(" + std::to_string(V(i,0)) + " , " + std::to_string(V(i,1)) + ")");
		}
		
		igl::opengl::glfw::imgui::ImGuiMenu menu;
		viewer.plugins.push_back(&menu);
		viewer.data().set_edges(V, E, C);
		viewer.data().show_lines = true;
		viewer.data().show_overlay = true;
		viewer.data().line_width = 2.0f;
		viewer.launch();
	}
	else if (mode ==4 ){
		ak::RowColGraph g;
		int courses = 30;
		int wales = 30;
		double radius = 1;
		double height = 6;
		g.vertices.resize(courses * wales);
		for (int i = 0; i < courses; ++i) {
			for (int j = 0; j < wales; ++j) {
				int v = i * wales + j;
				int l = i * wales + (j + (wales - 1)) % wales;
				int r = i * wales + (j + 1) % wales;
				int u = (i + 1) * wales + j;
				int b = (i - 1) * wales + j;

				if (u < g.vertices.size()) {
					g.vertices[v].add_col_out(u);
				}

				if (b >= 0) {
					g.vertices[v].add_col_in(b);
				}

				g.vertices[v].row_in = l;
				g.vertices[v].row_out = r;
				double x, y, z;

				x = radius * cos((double)j / wales);
				y = radius * sin((double) j / wales);
				z = height * (double)i / courses;

				g.vertices[v].at(0) = x;
				g.vertices[v].at(1) = y;
				g.vertices[v].at(2) = z;
			}
		}

		std::vector<ak::TracedStitch> traced_stitches;
		ak::trace_graph(g, &traced_stitches);
		ak::save_traced("traced.st", traced_stitches);

		std::vector<ak::Stitch> loaded_stitches;
	}
	else if (mode == 5) {

		CoarseKnitGraph G_c;

		std::vector<std::vector<std::vector<int>>> sides{
			{{5},{5},{5},{5}},
			{{5},{5},{5},{5}},
			{{5},{5},{5},{5}}
		};

		Eigen::MatrixXd corners(12, 3);
		corners <<
			-.5, 0, 0,
			.5, 0, 0,
			.5, 1, 0,
			-.5, 1, 0,

			.5, 0, 0,
			0, 0, -1,
			0, 1, -1,
			.5, 1, 0,

			0, 0, -1,
			-.5, 0, 0,
			-.5, 1, 0,
			0, 1, -1;
		
		Eigen::MatrixXd corners_0 = corners.block(0, 0, 4, 3);
		Eigen::MatrixXd corners_1 = corners.block(4, 0, 4, 3);
		Eigen::MatrixXd corners_2 = corners.block(8, 0, 4, 3);
		
		G_c.patches.emplace_back(sides[0], corners_0, 1, 1);
		G_c.patches.emplace_back(sides[1], corners_1, 1, 1);
		G_c.patches.emplace_back(sides[2], corners_2, 1, 1);

		G_c.edges.resize(3);
		G_c.edges[0].src = 0;
		G_c.edges[0].dst = 1;
		G_c.edges[0].src_side = 1;
		G_c.edges[0].dst_side = 3;
		G_c.edges[1].src = 1;
		G_c.edges[1].dst = 2;
		G_c.edges[1].src_side = 1;
		G_c.edges[1].dst_side = 3;
		G_c.edges[2].src = 2;
		G_c.edges[2].dst = 0;
		G_c.edges[2].src_side = 1;
		G_c.edges[2].dst_side = 3;

		KnitGraph G = G_c.build_graph();

		G.contract();

		Eigen::MatrixXd V;
		Eigen::MatrixXi E;
		Eigen::MatrixXd C;

		G.build_mesh(0.1, 4, V, E, C);

		ak::RowColGraph RCG = G.make_row_col_graph();
		std::vector<ak::TracedStitch> traced_stitches;
		ak::trace_graph(RCG, &traced_stitches);

		igl::opengl::glfw::Viewer viewer;
		viewer.data().set_points(V, Eigen::RowVector3d(1.0, 1.0, 1.0));
		for (int i = 0; i < V.rows(); ++i) {
			viewer.data().add_label(V.row(i), std::to_string(i));
		}

		igl::opengl::glfw::imgui::ImGuiMenu menu;
		viewer.plugins.push_back(&menu);
		viewer.data().set_edges(V, E, C);
		viewer.data().show_lines = true;
		viewer.data().show_overlay = true;
		viewer.data().line_width = 2.0f;
		viewer.launch();
	}
	else {
		std::vector<std::vector<int>> sides{ {3},{3},{3},{3} };
		Eigen::MatrixXd corners(4, 3);
		corners <<
			0, 0, 0,
			1, 0, 0,
			1, 1, 0,
			0, 1, 0;
		Patch p(sides, corners, 1, 1);
		p.graph.contract();
		p.graph.generate_instructions("test_trace");

		/*
		vkmp::Scheduler s;
		std::vector<vkmp::Stitch>& stitches = s.stitches;
		for (int r = 0; r < 5; ++r) {
			for (int c = 0; c < 5; ++c) {
			}
		}
		*/


	}

    return 0;
}
