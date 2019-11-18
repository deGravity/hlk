#define IGL_VIEWER_VIEWER_QUIET 1
// This does nothing during static compilation - will need to add it to the
// CMAKE for it to stick
#include <iostream>

#include "labeling_ui.h"
#include "remeshing_plugin.h"

using namespace hlk;

inline uint32_t str_to_int32_t(const std::string& str) {
    char* end_ptr = nullptr;
    int32_t result = (int32_t)strtol(str.c_str(), &end_ptr, 10);
    if (*end_ptr != '\0')
        throw std::runtime_error("Could not parse signed integer \"" + str + "\"");
    return result;
}

int main(int argc, char* argv[]) {
    std::vector<std::string> args;
    bool help = false;
    int rosy = 4;
    std::string input_path = "./tmp_in.obj";
    std::string output_path = "./tmp_out.obj";
    std::string input_model = "";

    try {
        for (int i = 1; i < argc; ++i) {
            if (strcmp("--help", argv[i]) == 0 || strcmp("-h", argv[i]) == 0) {
                help = true;
            } else if (strcmp("--rosy", argv[i]) == 0 || strcmp("-r", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing rotation symmetry type!\n";
                    return -1;
                }
                rosy = str_to_int32_t(argv[i]);
                if ((rosy != 2 && rosy != 4 && rosy != 6)) {
                    std::cerr << "Error: Invalid symmetry type!\n";
                    help = true;
                }
            } else if (strcmp("--input_path", argv[i]) == 0 || strcmp("-i", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing input file argument!\n";
                    return -1;
                }
                input_path = argv[i];
            } else if (strcmp("--output_path", argv[i]) == 0 || strcmp("-o", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing output file argument!\n";
                    return -1;
                }
                output_path = argv[i];
            } else if (strcmp("--input_model", argv[i]) == 0 || strcmp("-m", argv[i]) == 0) {
                if (++i >= argc) {
                    std::cerr << "Missing input model argument!\n";
                    return -1;
                }
                input_model = argv[i];
            } else {
                if (strncmp(argv[i], "-", 1) == 0) {
                    std::cerr << "Invalid argument: \"" << argv[i] << "\"!\n";
                    help = true;
                }
                args.push_back(argv[i]);
            }
        }
    } catch (const std::exception & e) {
        std::cout << "Error: " << e.what() << "\n";
        help = true;
    }

    if (args.size() > 1 || help || ((output_path.empty() || input_path.empty()) && args.size() == 0)) {
        std::cout << "Syntax: " << argv[0] << " [options] <input mesh / point cloud / application state snapshot>\n";
        std::cout << "Options:\n";
        std::cout << "   -i, --input_path       Writes to the specified PLY/OBJ file path which is the input for libQEx\n";
        std::cout << "   -o, --output_path      Writes to the specified PLY/OBJ file path which is the output for libQEx\n";
        std::cout << "   -r, --rosy <number>    Specifies the orientation symmetry type (2, 4, or 6)\n";
        std::cout << "   -m, --input_model      Specifies the input model that we want to preload into the UI\n";
        std::cout << "   -h, --help             Display this message\n";
        return -1;
    }

    if (args.size() == 0) { std::cout << "Running in GUI mode.\n"; }

    int mode;
    std::cout << "Choose an Interface\n"
              << "1) Remeshing\n"
              << "2) Labeling\n";

#ifdef HAISEN
    mode = 1;
#else
    std::cin >> mode;
#endif

    igl::opengl::glfw::Viewer viewer;
    if (mode == 1) {
        RemeshingMenu remeshing_menu(rosy, input_path, output_path);
        try {
            if (!input_model.empty()) {
                remeshing_menu.set_input_model(input_model);
            }
            viewer.plugins.push_back(&remeshing_menu);
            viewer.launch();
        } catch (const std::runtime_error & e) {
            std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
            if (remeshing_menu.save_workspace()) {
                std::cout << "all work saved.\n";
            } else {
                std::cout << "failed to save state...\n";
            }
            return -1;
        }
    } else {
        LabelingUI labelingui;
        viewer.plugins.push_back(&labelingui);
        viewer.launch();
    }

    return 0;
}
