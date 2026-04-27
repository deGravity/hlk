#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>

#include "symmetrizer.h"

using namespace Eigen;
using namespace std;

namespace hlk {

bool find_symmetric_vertices_along_axis(
    const MatrixXd& V, 
    const Vector3d& center, 
    int axis, 
    double epsilon, 
    VectorXi& V_symmetries);

bool find_symmetric_faces_along_axis(
    const MatrixXi& F,
    const VectorXi& V_symmetries,
    const vector<vector<int>>& adj,
    const vector<vector<int>>& adj_idx,
    VectorXi& F_symmetries);

tuple<int, int, int> discretize_vector(const Vector3d& v, double epsilon);

Symmetrizer::Symmetrizer(const MatrixXd& V, const MatrixXi& F) {
    // Find the center point
    Vector3d min_corner = V.colwise().minCoeff();
    Vector3d max_corner = V.colwise().maxCoeff();
    Vector3d center = (min_corner + max_corner) / 2;
    vector<vector<int>> adj(V.rows());
    vector<vector<int>> adj_idx(V.rows());

    // Find the minimum edge lengths and set epsilon to be half of that.
    // Also calculate vertex-triangle adjacencies for later processing.
    double min_edge_length = (max_corner - min_corner).norm();
    double avg_edge_length = 0;
    for (int f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            avg_edge_length += (V.row(F(f, k)) - V.row(F(f, (k + 1) % 3))).norm();
            min_edge_length = min((V.row(F(f, k)) - V.row(F(f, (k + 1) % 3))).norm(), min_edge_length);
            adj[F(f, k)].push_back(f);
            adj_idx[F(f, k)].push_back(k);
        }
    }
    double epsilon = avg_edge_length / (F.rows() * 3);

    // Attempt to find symmetric vertices and faces along each axis
    for (int i = 0; i < 3; ++i) {
        face_symmetry[i] = false;
        vertex_symmetry[i] = find_symmetric_vertices_along_axis(V, center, i, epsilon, V_symmetries[i]);
        if (vertex_symmetry[i]) {
            face_symmetry[i] = find_symmetric_faces_along_axis(F, V_symmetries[i], adj, adj_idx, F_symmetries[i]);
        }
    }
}

void Symmetrizer::symmetrize(Matrix<double, -1, -1>& f, bool axes[3]) {
    for (int i = 0; i < 3; ++i) {
        if (axes[i] && face_symmetry[i]) {
            symmetrize(f, i);
        }
    }
}

void Symmetrizer::symmetrize(Matrix<double, -1, -1>& f, int axis) {
    // Figure out if this is a scalar or vector valued function, 
    // and if it is face- or vertex- defined
    if (f.rows() == V_symmetries[0].size()) {
        cout << "Symmetrizing by vertex along axis " << axis << endl;
        symmetrize_by_vertex(f, axis);
    } else if (f.rows() == F_symmetries[0].size()) {
        symmetrize_by_face(f, axis);
    } else {
        assert(false); // Incorrectly dimensioned input
    }
}

void Symmetrizer::symmetrize_by_vertex(Matrix<double, -1, -1>& f, int axis) {
    assert(face_symmetry[axis]);
    for (int i = 0; i < f.rows(); ++i) {
        VectorXd average(f.cols());
        VectorXd mirrored = f.row(V_symmetries[axis][i]);
        if (f.cols() == 3) {
            mirrored(axis) = -mirrored(axis);
        }
        average = (f.row(i) + mirrored) / 2;
        f.row(i) = average;
        if (f.cols() == 3) {
            average(axis) = -average(axis);
        }
        f.row(V_symmetries[axis][i]) = average;
    }
}

void Symmetrizer::symmetrize_by_face(Matrix<double, -1, -1>& f, int axis) {
    assert(face_symmetry[axis]);
    for (int i = 0; i < f.rows(); ++i) {
        VectorXd main_vec = f.row(i);
        int symmetric_face = F_symmetries[axis][i];
        VectorXd mirrored = f.row(symmetric_face);
        if (f.cols() == 3) {
            mirrored(axis) = -mirrored(axis);
        }
        VectorXd average = (main_vec + mirrored) / 2;
        f.row(i) = average;
        if (f.cols() == 3) {
            average(axis) = -average(axis);
        }
        f.row(symmetric_face) = average;
    }
}

void Symmetrizer::symmetrize_geometry(Eigen::MatrixXd& V) {
    VectorXi is_set(V.rows());
    Vector3d center = (V.colwise().minCoeff() + V.colwise().maxCoeff()) / 2;

    for (int k = 0; k < 3; ++k) {
        if (!vertex_symmetry[k]) continue;

        is_set.setConstant(false);
        for (int v = 0; v < V.rows(); ++v) {
            if (is_set(v)) continue;

            int v_opp = V_symmetries[k][v];
            Vector3d v_opp_pos = V.row(v);
            v_opp_pos(k) = 2 * center(k) - v_opp_pos(k);
            V.row(v_opp) = v_opp_pos;

            is_set(v) = true;
            is_set(v_opp) = true;
        }
    }
}

void Symmetrizer::symmetrize_topology(const Eigen::MatrixXd& V, Eigen::MatrixXi& F, int axis) {
    assert(vertex_symmetry[axis]);
    Vector3d center = (V.colwise().minCoeff() + V.colwise().maxCoeff()) / 2;

    vector<int> left_faces;
    vector<int> right_faces;

    for (int f = 0; f < F.rows(); ++f) {
        bool is_left_face = true;
        for (int k = 0; k < 3; ++k) {
            int v = F(f, k);
            int v_opp = V_symmetries[axis][v];
            Vector3d v_pos = V.row(v);
            if (v != v_opp && v_pos(axis) > center(axis)) {
                is_left_face = false;
                break;
            }
        }
        if (is_left_face) {
            left_faces.push_back(f);
        } else {
            right_faces.push_back(f);
        }
    }

    // Make sure the underlying mesh was actually symmetric
    assert(left_faces.size() == right_faces.size());

    for (int i = 0; i < left_faces.size(); ++i) {
        int f_l = left_faces[i];
        int f_r = right_faces[i];
        Vector3i f = F.row(f_l);
        F.row(f_r) = Vector3i(
            V_symmetries[axis][f(2)],
            V_symmetries[axis][f(1)],
            V_symmetries[axis][f(0)]
        );
    }

}

vector<int> Symmetrizer::symmetric_faces(int f) {
    return symmetric_faces(f, {0,1,2});
}

vector<int> Symmetrizer::symmetric_faces(int f, const vector<int>& axes) {
    return symmetric_elements(F_symmetries, face_symmetry, axes, f);
}

vector<int> Symmetrizer::symmetric_vertices(int v) {
    return symmetric_vertices(v, {0,1,2});
}

vector<int> Symmetrizer::symmetric_vertices(int v, const vector<int>& axes) {
    return symmetric_elements(V_symmetries, vertex_symmetry, axes, v);
}

int Symmetrizer::symmetric_face(int f, int axis) {
    assert(face_symmetry[axis]);
    return F_symmetries[axis][f];
}

int Symmetrizer::symmetric_vertex(int v, int axis) {
    assert(vertex_symmetry[axis]);
    return V_symmetries[axis][v];
}

bool Symmetrizer::has_face_symmetry() {
    return face_symmetry[0] || face_symmetry[1] || face_symmetry[2];
}

bool Symmetrizer::has_face_symmetry(int axis) {
    assert(axis < 3 && axis >= 0);
    return face_symmetry[axis];
}

bool Symmetrizer::has_vertex_symmetry() {
    return vertex_symmetry[0] || vertex_symmetry[1] || vertex_symmetry[2];
}

bool Symmetrizer::has_vertex_symmetry(int axis) {
    return vertex_symmetry[axis];
}

vector<int> Symmetrizer::symmetric_elements(const VectorXi symmetries[], bool symmetry_checks[], int index) {
    return symmetric_elements(symmetries, symmetry_checks, { 0, 1, 2 }, index);
}

vector<int> Symmetrizer::symmetric_elements(const VectorXi symmetries[], bool symmetry_checks[], const vector<int>& axes, int index) {
    set<int> elements;
    elements.insert(index);
    for (int k : axes) {
        if (symmetry_checks[k]) {
            for (auto idx : elements) {
                elements.insert(symmetries[k][idx]);
            }
        }
    }
    vector<int> rv;
    for (auto idx : elements) {
        rv.push_back(idx);
    }
    return rv;
}

////////////////////
// Helper methods //
////////////////////

bool find_symmetric_vertices_along_axis(
    const MatrixXd& V, 
    const Vector3d& center, 
    int axis, 
    double epsilon, 
    VectorXi& V_symmetries) {

    map<tuple<int, int, int>, vector<pair<int, Vector3d>>> mirrored_vertices;
    for (int i = 0; i < V.rows(); ++i) {
        Vector3d v = V.row(i);
        Vector3d diff = center - v;
        for (int k = 0; k < 3; ++k) {
            if (k != axis) {
                diff(k) = 0;
            }
        }
        Vector3d v_mirrored = v + 2 * diff;
        mirrored_vertices[make_tuple(0,0,0)].push_back(make_pair(i, v_mirrored));
    }

    V_symmetries.resize(V.rows());
    bool has_symmetry = true;
    for (int i = 0; i < V.rows(); ++i) {
        Vector3d v = V.row(i);
        int best_match = -1;
        double best_match_distance = numeric_limits<double>::max();
        tuple<int, int, int> query = discretize_vector(v, epsilon);
        for (auto& match : mirrored_vertices[make_tuple(0,0,0)]) {
            int idx = match.first;
            const Vector3d& pos = match.second;
            double dist = (v - pos).norm();
            if (dist < best_match_distance) {
                best_match = idx;
                best_match_distance = dist;
            }
        }
        
        V_symmetries[i] = best_match;
        if (best_match_distance > epsilon) {
            has_symmetry &= false;
            break;
        }
    }
    return has_symmetry;
}

bool find_symmetric_faces_along_axis(
    const MatrixXi& F, 
    const VectorXi& V_symmetries, 
    const vector<vector<int>>& adj, 
    const vector<vector<int>>& adj_idx, 
    VectorXi& F_symmetries)  {

    bool has_symmetry = true;
    F_symmetries.resize(F.rows());
    for (int f0 = 0; f0 < F.rows(); ++f0) {
        int v0 = F(f0, 0);
        int v1 = F(f0, 1);
        int v2 = F(f0, 2);
        int v0_opp = V_symmetries[v0];
        int v1_opp = V_symmetries[v1];
        int v2_opp = V_symmetries[v2];
        int match_idx = -1;

        for (int i = 0; i < adj[v0_opp].size(); ++i) {
            int f1 = adj[v0_opp][i];
            int opp_idx = adj_idx[v0_opp][i];
            Vector3i candidate_face = F.row(f1);
            // Check the two other faces in the candidate triangle
            // Since this is a mirror symmetry, ordering is reversed
            if (v2_opp == F(f1, (opp_idx + 1) % 3) && 
                v1_opp == F(f1, (opp_idx + 2) % 3)) {
                match_idx = f1;
                break;
            }
        }

        F_symmetries[f0] = match_idx;
        if (match_idx == -1) {
            has_symmetry &= false;
            break;
        }
    }
    return has_symmetry;
}

tuple<int, int, int> discretize_vector(const Vector3d& v, double epsilon) {
    Vector3d scaled = v / epsilon;
    return make_tuple<int, int, int>((int) nearbyint(scaled(0)),(int) nearbyint(scaled(1)), (int) nearbyint(scaled(2)));
}

}
