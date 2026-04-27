#define IGL_VIEWER_VIEWER_QUIET 1

#include <iostream>
#include <string>

#include <igl/opengl/glfw/Viewer.h>

#include "labeling_ui.h"

using namespace hlk;

int main(int argc, char* argv[]) {
    std::string input_quad_obj;
    bool help = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--help" || a == "-h") help = true;
        else if (a == "--input" || a == "-i") {
            if (++i >= argc) { std::cerr << "Missing value for --input\n"; return 1; }
            input_quad_obj = argv[i];
        }
        else if (!a.empty() && a[0] == '-') {
            std::cerr << "Unknown option: " << a << "\n";
            help = true;
        }
        else {
            input_quad_obj = a;
        }
    }

    if (help) {
        std::cout <<
            "Usage: " << argv[0] << " [options] [quad-mesh.obj]\n"
            "  -i, --input PATH   Quad-mesh OBJ to load on startup (typically the\n"
            "                     output of hlk_remesh).\n"
            "  -h, --help         Show this message\n";
        return 0;
    }

    igl::opengl::glfw::Viewer viewer;
    LabelingUI labeling_ui;
    viewer.plugins.push_back(&labeling_ui);

    if (!input_quad_obj.empty()) {
        // Auto-load is wired up in Phase 3 (LabelingUI's load_quad_mesh_file
        // touches viewer state, so it has to run after viewer.launch()).
        // For now, log and let the user open via File > Open.
        std::cout << "[hlk_label] Note: auto-load not yet wired; "
                  << "use File > Open Quad Mesh to load \""
                  << input_quad_obj << "\".\n";
    }

    viewer.launch();
    return 0;
}
