#pragma once

#include "quad_mesh.h"
#include <string>


namespace hlk {
	void read_quad_mesh(const std::string& obj_file, QuadMesh& Q);
}