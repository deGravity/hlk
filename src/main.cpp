#define IGL_VIEWER_VIEWER_QUIET 1
// This does nothing during static compilation - will need to add it to the
// CMAKE for it to stick
#include <iostream>

#include "labeling_ui.h"
#include "remeshing_plugin.h"

using namespace hlk;

int main(void) {

	while (true) {
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

		remeshing_menu.quad_mesh;

		igl::opengl::glfw::Viewer viewer2;
		LabelingUI labelingui;
		viewer2.plugins.push_back(&labelingui);
		viewer2.launch();

	}
    return 0;
}
