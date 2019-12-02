#define IGL_VIEWER_VIEWER_QUIET 1
// This does nothing during static compilation - will need to add it to the
// CMAKE for it to stick
#include <iostream>

#include "labeling_ui.h"
#include "remeshing_plugin.h"
#include "patch.h"

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
	else { // test patch visualization
		//std::string filename = igl::file_dialog_open();
		std::vector<std::vector<int>> sides{ {3,2},{2},{5},{5} };
		Eigen::MatrixXd corners(5, 3);
		corners << 
			1, 0, 0,
			1.3, 0.5, 0,
			1, 1, 0,
			0, 1, 0,
			0, 0, 0;
		Patch p(sides, corners);

		Eigen::MatrixXd V;
		Eigen::MatrixXi E;
		Eigen::MatrixXd C;
		


		p.interpolate_coordinates();
		p.graph.build_mesh(0.1, 4, V, E, C);

		std::cout << V << std::endl << std::endl << E << std::endl;

		igl::opengl::glfw::Viewer viewer;
		viewer.data().set_points(V, Eigen::RowVector3d(1.0, 1.0, 1.0));
		viewer.data().set_edges(V, E, C);
		viewer.data().show_lines = true;
		viewer.data().show_overlay = true;
		viewer.data().line_width = 2.0f;
		viewer.launch();
	}

    return 0;
}
