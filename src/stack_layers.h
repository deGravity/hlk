#pragma once

#include <Eigen/Dense>

namespace hlk {
    void offset_layers(
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXd& N,
        const Eigen::VectorXi& layer,
        Eigen::MatrixXd& V_out
    );
}
