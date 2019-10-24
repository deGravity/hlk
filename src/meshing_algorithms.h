#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hlk {

class Meshing {
public:
    // LOOKHERE
    static void polyvector_parametrize();

    static void frame_field_miq(
        const Eigen::MatrixXd& X1, /* deformed first representative */
        const Eigen::MatrixXd& X2, /* deformed second representative */
        const Eigen::MatrixXd& V_deformed, const Eigen::MatrixXi& F,
        double gradient_size, double stiffness,
        Eigen::MatrixXd& UV, Eigen::MatrixXi& FUV);

    static void cross_field_miq(
        const Eigen::MatrixXd& R, /* N-Rosy field */
        const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
        const std::vector<int>& vertices_to_round, 
        const std::vector<std::vector<int>>& hard_edges,
        double gradient_size, double stiffness,
        Eigen::MatrixXd& UV, Eigen::MatrixXi& FUV);

    static void init_curl(
        const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh, const int N, // nrosy's n
        Eigen::MatrixXi& EV, Eigen::MatrixXi& EF, Eigen::MatrixXi& FE,
        Eigen::VectorXi& matching, Eigen::VectorXi& combedMatching,
        Eigen::VectorXd& effort, Eigen::VectorXd& combedEffort,
        Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
        Eigen::VectorXd& curl, Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
        Eigen::SparseMatrix<double>& AE2F, double& curlMax, double& curlMaxOrig);

    static void reduce_curl(
        const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh, const int N, // nrosy's n
        const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
        Eigen::VectorXi& matching, Eigen::VectorXi& combedMatching,
        Eigen::VectorXd& effort, Eigen::VectorXd& combedEffort,
        Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
        Eigen::VectorXd& curl, Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
        double& curlMax);
};

}
