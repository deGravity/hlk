#pragma once
//
// directional_ext.h — extensions to the directional namespace that the
// meshing-branch remeshing UI relies on but that are not yet upstreamed.
//
//  - directional::dual_cycles(...) 11-arg overload: appends rows to
//    basisCycles for direction constraints per Crane, Desbrun, Schröder,
//    "Trivial Connections on Discrete Surfaces" (SGP 2010), §2.8.
//
//  - directional::index_prescription(...) overloads that additionally
//    output a per-cycle |basisCycles*x − b| residual vector (`linf`)
//    used by the remeshing UI to color-visualize cycle errors.
//
// Both are header-only inline functions to avoid ODR issues; including
// this header in multiple translation units is safe.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <igl/local_basis.h>
#include <igl/PI.h>

#include <directional/dual_cycles.h>
#include <directional/index_prescription.h>

namespace directional {
namespace hlk_ext {

// Parallel-transport an angle alpha from face_from to face_to across `edge`,
// using face local bases B1/B2 (igl::local_basis). Equation (1) in Crane et al.
inline double parallel_transport_angle(
    double alpha,
    int face_from, int face_to, int edge,
    const Eigen::MatrixXd& V, const Eigen::MatrixXi& EV,
    const Eigen::MatrixXd& B1, const Eigen::MatrixXd& B2)
{
    Eigen::Vector3d e = (V.row(EV(edge, 1)) - V.row(EV(edge, 0))).transpose().normalized();
    Eigen::Vector3d B1f = B1.row(face_from).transpose();
    Eigen::Vector3d B2f = B2.row(face_from).transpose();
    Eigen::Vector3d B1t = B1.row(face_to).transpose();
    Eigen::Vector3d B2t = B2.row(face_to).transpose();
    double theta_from = std::atan2(e.dot(B2f), e.dot(B1f));
    double theta_to   = std::atan2(e.dot(B2t), e.dot(B1t));
    return alpha - theta_from + theta_to;
}

// Reduce x to the equivalent representative in [-period/2, period/2).
inline double wrap_to_principal(double x, double period) {
    const double half = 0.5 * period;
    double y = std::fmod(x + half, period);
    if (y < 0) y += period;
    return y - half;
}

} // namespace hlk_ext

//
// dual_cycles overload with direction constraints.
//
// First runs the standard dual_cycles to populate basisCycles / cycleCurvature
// / vertex2cycle / innerEdges, then appends one new row to basisCycles per
// constraint cycle. A constraint cycle is the dual-edge path between two
// constrained faces in a primal-face spanning tree rooted at face_inds[0].
// The corresponding entry in cycleCurvature is the angular discrepancy
// between the prescribed direction in the ancestor face and the parallel-
// transported direction from the descendant face, taken modulo 2π/N so the
// resulting connection is consistent for an N-RoSy field.
//
inline void dual_cycles(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXi& EV,
    const Eigen::MatrixXi& EF,
    Eigen::SparseMatrix<double>& basisCycles,
    Eigen::VectorXd& cycleCurvature,
    Eigen::VectorXi& vertex2cycle,
    Eigen::VectorXi& innerEdges,
    const std::vector<int>& face_inds,
    const std::vector<double>& constraint_angles,
    int N)
{
    using namespace Eigen;

    // 1. Standard topological cycles.
    dual_cycles(V, F, EV, EF, basisCycles, cycleCurvature, vertex2cycle, innerEdges);

    if (face_inds.size() < 2) return;  // need ≥ 2 constrained faces to form a cycle

    // 2. Local bases per face for parallel transport.
    MatrixXd B1, B2, B3;
    igl::local_basis(V, F, B1, B2, B3);

    // face index → position in face_inds (or -1 if not constrained)
    std::vector<int> face_constraint_idx((size_t)F.rows(), -1);
    for (size_t i = 0; i < face_inds.size(); ++i) {
        face_constraint_idx[(size_t)face_inds[i]] = (int)i;
    }

    // edge index → column in basisCycles (== position in innerEdges).
    std::unordered_map<int, int> edge_to_col;
    edge_to_col.reserve((size_t)innerEdges.size());
    for (int i = 0; i < innerEdges.size(); ++i) {
        edge_to_col[innerEdges(i)] = i;
    }

    // 3. Face-face adjacency (for BFS over primal faces).
    std::vector<std::vector<std::pair<int,int>>> face_neighbors((size_t)F.rows());
    for (int e = 0; e < EV.rows(); ++e) {
        int f0 = EF(e, 0), f1 = EF(e, 1);
        if (f0 < 0 || f1 < 0) continue;  // boundary
        face_neighbors[(size_t)f0].push_back({e, f1});
        face_neighbors[(size_t)f1].push_back({e, f0});
    }

    // 4. BFS from face_inds[0]; track parent face / edge.
    std::vector<int> parent_face((size_t)F.rows(), -1);
    std::vector<int> parent_edge((size_t)F.rows(), -1);
    std::vector<char> visited((size_t)F.rows(), 0);

    int root = face_inds[0];
    visited[(size_t)root] = 1;
    std::queue<int> Q;
    Q.push(root);

    std::vector<std::vector<Triplet<double>>> new_cycle_triplets;
    std::vector<double> new_holonomies;
    int next_cycle_row = (int)basisCycles.rows();

    while (!Q.empty()) {
        int f = Q.front(); Q.pop();
        for (const auto& kv : face_neighbors[(size_t)f]) {
            int e = kv.first;
            int g = kv.second;
            if (visited[(size_t)g]) continue;
            visited[(size_t)g] = 1;
            parent_face[(size_t)g] = f;
            parent_edge[(size_t)g] = e;
            Q.push(g);

            if (face_constraint_idx[(size_t)g] < 0 || g == root) continue;

            // 5. Walk g → ancestor along the BFS tree, collecting dual edges
            //    and parallel-transporting g's constraint angle as we go.
            std::vector<Triplet<double>> cycle_trips;
            double alpha = constraint_angles[(size_t)face_constraint_idx[(size_t)g]];
            int h = g;
            while (true) {
                int p = parent_face[(size_t)h];
                int e_hp = parent_edge[(size_t)h];
                // Dual edge orientation convention: +1 if traversal goes
                // EF(e,0)→EF(e,1); else -1. We're going h → p.
                double sign = (EF(e_hp, 0) == h) ? +1.0 : -1.0;
                auto col_it = edge_to_col.find(e_hp);
                if (col_it != edge_to_col.end()) {
                    cycle_trips.emplace_back(next_cycle_row, col_it->second, sign);
                }
                alpha = hlk_ext::parallel_transport_angle(
                    alpha, h, p, e_hp, V, EV, B1, B2);
                h = p;
                if (face_constraint_idx[(size_t)h] >= 0) break;
            }

            // 6. Holonomy: difference between ancestor's prescribed angle
            //    and the transported angle, reduced mod 2π/N.
            double alpha_h = constraint_angles[(size_t)face_constraint_idx[(size_t)h]];
            double period = 2.0 * igl::PI / (double)N;
            double holonomy = hlk_ext::wrap_to_principal(alpha_h - alpha, period);

            new_cycle_triplets.emplace_back(std::move(cycle_trips));
            new_holonomies.push_back(holonomy);
            ++next_cycle_row;
        }
    }

    int n_new = (int)new_cycle_triplets.size();
    if (n_new == 0) return;

    // 7. Append the new rows to basisCycles by re-building from triplets.
    SparseMatrix<double> extended((Index)basisCycles.rows() + n_new, basisCycles.cols());
    std::vector<Triplet<double>> all_trips;
    all_trips.reserve((size_t)basisCycles.nonZeros() + (size_t)n_new * 8);
    for (int k = 0; k < basisCycles.outerSize(); ++k) {
        for (SparseMatrix<double>::InnerIterator it(basisCycles, k); it; ++it) {
            all_trips.emplace_back((int)it.row(), (int)it.col(), it.value());
        }
    }
    for (auto& cs : new_cycle_triplets) {
        for (auto& t : cs) all_trips.push_back(t);
    }
    extended.setFromTriplets(all_trips.begin(), all_trips.end());
    basisCycles = std::move(extended);

    VectorXd extended_curv(cycleCurvature.size() + n_new);
    extended_curv.head(cycleCurvature.size()) = cycleCurvature;
    for (int i = 0; i < n_new; ++i) {
        extended_curv(cycleCurvature.size() + i) = new_holonomies[(size_t)i];
    }
    cycleCurvature = std::move(extended_curv);
}

namespace hlk_ext {
// Compute the per-cycle |basisCycles*innerRotationAngles - b| residual.
inline void compute_cycle_residual(
    const Eigen::SparseMatrix<double>& basisCycles,
    const Eigen::VectorXi& innerEdges,
    const Eigen::VectorXd& cycleCurvature,
    const Eigen::VectorXi& cycleIndices,
    const Eigen::VectorXd& rotationAngles,
    int N,
    Eigen::VectorXd& linf)
{
    Eigen::VectorXd cycleNewCurvature =
        cycleIndices.cast<double>() * (2.0 * igl::PI / (double)N);
    Eigen::VectorXd inner(innerEdges.size());
    for (int i = 0; i < innerEdges.size(); ++i) {
        inner(i) = rotationAngles(innerEdges(i));
    }
    Eigen::VectorXd b = -cycleCurvature + cycleNewCurvature;
    linf = (basisCycles * inner - b).cwiseAbs();
}
} // namespace hlk_ext

// index_prescription overloads with per-cycle linf vector output.
//
// Soft-constraint variant (uses targetCurvature + lambda guidance weight,
// matching the collaborator's c353170 commit).
inline void index_prescription(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::VectorXi& innerEdges,
    const Eigen::SparseMatrix<double>& basisCycles,
    const Eigen::VectorXd& targetCurvature,
    const Eigen::VectorXd& cycleCurvature,
    const Eigen::VectorXi& cycleIndices,
    int N,
    double lambda,
    Eigen::VectorXd& rotationAngles,
    Eigen::VectorXd& linf,
    double& linfError)
{
    index_prescription(V, F, innerEdges, basisCycles, targetCurvature,
                       cycleCurvature, cycleIndices, N, lambda,
                       rotationAngles, linfError);
    hlk_ext::compute_cycle_residual(basisCycles, innerEdges, cycleCurvature,
                                    cycleIndices, rotationAngles, N, linf);
}

// Plain (no-guidance) variant.
inline void index_prescription(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::VectorXi& innerEdges,
    const Eigen::SparseMatrix<double>& basisCycles,
    const Eigen::VectorXd& cycleCurvature,
    const Eigen::VectorXi& cycleIndices,
    int N,
    Eigen::VectorXd& rotationAngles,
    Eigen::VectorXd& linf,
    double& linfError)
{
    index_prescription(V, F, innerEdges, basisCycles, cycleCurvature,
                       cycleIndices, N, rotationAngles, linfError);
    hlk_ext::compute_cycle_residual(basisCycles, innerEdges, cycleCurvature,
                                    cycleIndices, rotationAngles, N, linf);
}

} // namespace directional
