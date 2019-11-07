#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hlk {

class Meshing {
public:
    static void polyvector_parametrize(
        const Eigen::MatrixXd& VMeshWhole, const Eigen::MatrixXi& FMeshWhole, const int N,
        const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
        const Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
        Eigen::VectorXi& matching, Eigen::VectorXi& combedMatching,
        Eigen::VectorXd& effort, Eigen::VectorXd& combedEffort,
        Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
        Eigen::MatrixXd& VMeshCut, Eigen::MatrixXi& FMeshCut,
        Eigen::MatrixXd& cutUV, double lengthRatio = 0.1, bool isInteger = true);

    static void cross_field_miq(
        const Eigen::MatrixXd& X1, /* direction field */
        const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
        const std::vector<std::vector<int>>& hard_edges,
        double gradient_size, int stiffen_iter,
        Eigen::MatrixXd& UV, Eigen::MatrixXi& FUV);

    static void init_curl(
        const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh, const int N,
        const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
        const Eigen::VectorXi& b, const Eigen::MatrixXd& bc, const Eigen::VectorXi& blevel,
        const Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
        Eigen::VectorXi& matching, Eigen::VectorXi& combedMatching,
        Eigen::VectorXd& effort, Eigen::VectorXd& combedEffort,
        Eigen::VectorXd& curl, Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
        Eigen::SparseMatrix<double>& AE2F, double& curlMax, double& curlMaxOrig);

    static void reduce_curl(
        const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh, const int N,
        const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
        Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
        Eigen::VectorXi& matching, Eigen::VectorXi& combedMatching,
        Eigen::VectorXd& effort, Eigen::VectorXd& combedEffort,
        Eigen::VectorXd& curl, Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
        double& curlMax);
};

}
