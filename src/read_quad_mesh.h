#pragma once

#include <string>

#include "quad_mesh.h"

namespace hlk {
    void read_quad_mesh(const std::string& obj_file, QuadMesh& Q, bool planarize = false);
}
