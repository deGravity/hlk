#pragma once

#include <vector>

#include <Eigen/Dense>

namespace hlk {

class Symmetrizer {
public:
    Symmetrizer() {}
    Symmetrizer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

    void symmetrize(Eigen::Matrix<double, -1, -1>& f, bool axes[3]);
    void symmetrize(Eigen::Matrix<double, -1, -1>& f, int axis);
    void symmetrize_by_vertex(Eigen::Matrix<double, -1, -1>& f, int axis);
    void symmetrize_by_face(Eigen::Matrix<double, -1, -1>& f, int axis);
    void symmetrize_geometry(Eigen::MatrixXd& V);
    void symmetrize_topology(const Eigen::MatrixXd& V, Eigen::MatrixXi& F, int axis);

    std::vector<int> symmetric_faces(int f);
    std::vector<int> symmetric_faces(int f, const std::vector<int>& axes);
    int symmetric_face(int f, int axis);
    std::vector<int> symmetric_vertices(int v);
    std::vector<int> symmetric_vertices(int v, const std::vector<int>& axes);
    int symmetric_vertex(int v, int axis);

    bool has_face_symmetry();
    bool has_face_symmetry(int axis);
    bool has_vertex_symmetry();
    bool has_vertex_symmetry(int axis);

private:
    std::vector<int> symmetric_elements(const Eigen::VectorXi symmetries[],
        bool symmetry_checks[], int index);
    std::vector<int> symmetric_elements(const Eigen::VectorXi symmetries[], 
        bool symmetry_checks[], const std::vector<int>& axes, int index);

    bool face_symmetry[3];
    bool vertex_symmetry[3];

    Eigen::VectorXi V_symmetries[3];
    Eigen::VectorXi F_symmetries[3];
    std::vector<int> selected_faces;
};	

}
