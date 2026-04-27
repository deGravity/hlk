#define IGL_VIEWER_VIEWER_QUIET 1

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <igl/opengl/glfw/Viewer.h>

#include "unified_ui.h"

using namespace hlk;

static uint32_t parse_int(const std::string& str) {
    char* end = nullptr;
    int32_t result = (int32_t)std::strtol(str.c_str(), &end, 10);
    if (*end != '\0') throw std::runtime_error("Could not parse integer \"" + str + "\"");
    return result;
}

int main(int argc, char* argv[]) {
    int rosy = 4;
    std::string input_path = "./tmp_in.obj";
    std::string output_path = "./tmp_out.obj";
    std::string input_model;
    bool help = false;
    std::vector<std::string> positional;

    try {
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--help" || a == "-h") help = true;
            else if (a == "--rosy" || a == "-r") {
                if (++i >= argc) throw std::runtime_error("Missing value for --rosy");
                rosy = parse_int(argv[i]);
                if (rosy != 2 && rosy != 4 && rosy != 6)
                    throw std::runtime_error("Invalid --rosy value (must be 2, 4, or 6)");
            }
            else if (a == "--input_path" || a == "-i") {
                if (++i >= argc) throw std::runtime_error("Missing value for --input_path");
                input_path = argv[i];
            }
            else if (a == "--output_path" || a == "-o") {
                if (++i >= argc) throw std::runtime_error("Missing value for --output_path");
                output_path = argv[i];
            }
            else if (a == "--input_model" || a == "-m") {
                if (++i >= argc) throw std::runtime_error("Missing value for --input_model");
                input_model = argv[i];
            }
            else if (!a.empty() && a[0] == '-') {
                throw std::runtime_error("Unknown option: " + a);
            }
            else {
                positional.push_back(a);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        help = true;
    }

    if (help) {
        std::cout <<
            "Usage: " << argv[0] << " [options] [input mesh]\n"
            "  Single binary covering the full hlk pipeline. Starts in the\n"
            "  Remesh stage; switch to Label via the Stage panel after a\n"
            "  quad mesh has been extracted.\n"
            "\n"
            "  -i, --input_path PATH   Intermediate input OBJ for libQEx (default: ./tmp_in.obj)\n"
            "  -o, --output_path PATH  Output quad OBJ (default: ./tmp_out.obj)\n"
            "  -r, --rosy {2,4,6}      Rotation symmetry (default: 4)\n"
            "  -m, --input_model PATH  Preload this triangle mesh into the Remesh UI\n"
            "  -h, --help              Show this message\n";
        return 0;
    }

    if (input_model.empty() && !positional.empty()) {
        input_model = positional.front();
    }

    igl::opengl::glfw::Viewer viewer;
    UnifiedUI ui(rosy, input_path, output_path);
    if (!input_model.empty()) ui.remesh().set_input_model(input_model);
    viewer.plugins.push_back(&ui);

    try {
        viewer.launch();
    } catch (const std::runtime_error& e) {
        std::cerr << "Caught a fatal error: " << e.what() << "\n";
        if (ui.remesh().save_workspace()) std::cout << "Workspace saved.\n";
        else std::cout << "Failed to save workspace.\n";
        return 1;
    }

    return 0;
}
