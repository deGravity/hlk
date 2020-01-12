#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hlk {

class Meshing {
public:
    static void comb_field_from_connection(
        const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh,
        const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
        const Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
        Eigen::VectorXi& combedMatching, Eigen::VectorXd& combedEffort);

    static void cross_field_miq(
        const Eigen::MatrixXd& X1, /* direction field */
        const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
        const std::vector<std::vector<int>>& hard_edges,
        double gradient_size, int stiffen_iter,
        Eigen::MatrixXd& UV, Eigen::MatrixXi& FUV);
};

}
