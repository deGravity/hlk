#pragma once

#include "quad_mesh.h"

#include "Eigen/Core"

namespace hlk {
	void extract_quad_mesh(
		const Eigen::MatrixXd& V,
		const Eigen::MatrixXi& F,
		const Eigen::MatrixXd& TC,
		const Eigen::MatrixXi& FTC,
		QuadMesh& Q);
}
