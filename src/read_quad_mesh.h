#pragma once

#include <string>

#include "quad_mesh.h"

#include <Eigen/Core>

namespace hlk {
    // Load a quad mesh from an OBJ file. Removes non-quad faces and
    // unreferenced vertices, optionally planarizes, and runs Q.init().
    void read_quad_mesh(const std::string& obj_file, QuadMesh& Q, bool planarize = false);

    // In-memory equivalent of read_quad_mesh. Use this when the source of
    // V/F is something other than disk — e.g., the in-process handoff from
    // hlk_remesh's quad-extraction output to hlk_label.
    void load_quad_mesh_in_memory(const Eigen::MatrixXd& V_in,
                                  const Eigen::MatrixXi& F_in,
                                  QuadMesh& Q,
                                  bool planarize = false);
}
