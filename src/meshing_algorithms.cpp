#include <directional/combing.h>
#include <directional/cut_mesh_with_singularities.h>
#include <directional/curl_matching.h>
#include <directional/effort_to_indices.h>
#include <directional/parameterize.h>
#include <directional/principal_matching.h>
#include <directional/polycurl_reduction.h>
#include <igl/avg_edge_length.h>
#include <igl/barycenter.h>
#include <igl/comb_cross_field.h>
#include <igl/comb_frame_field.h>
#include <igl/compute_frame_field_bisectors.h>
#include <igl/copyleft/comiso/miq.h>
#include <igl/cross_field_mismatch.h>
#include <igl/cut_mesh_from_singularities.h>
#include <igl/edge_topology.h>
#include <igl/find_cross_field_singularities.h>
#include <igl/rotate_vectors.h>
#include <igl/PI.h>

#include "meshing_algorithms.h"

namespace hlk {
void Meshing::comb_field_from_connection(
    const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh,
    const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
    const Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
    Eigen::VectorXi& combedMatching, Eigen::VectorXd& combedEffort) {

    // combing
    Eigen::VectorXi matching;
    Eigen::VectorXd effort;
    directional::principal_matching(VMesh, FMesh, EV, EF, FE, rawField, matching, effort);
    directional::combing(VMesh, FMesh, EV, EF, FE, rawField, matching, combedField);
    directional::principal_matching(VMesh, FMesh, EV, EF, FE, combedField, combedMatching, combedEffort);
}

void Meshing::polyvector_parametrize(
    const Eigen::MatrixXd& VMeshWhole, const Eigen::MatrixXi& FMeshWhole, const int N,
    const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
    const Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
    Eigen::VectorXi& combedMatching, Eigen::VectorXd& combedEffort,
    Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
    Eigen::MatrixXd& VMeshCut, Eigen::MatrixXi& FMeshCut,
    Eigen::MatrixXd& cutUV, double lengthRatio, bool isInteger) {

    // combing and cutting
    Eigen::VectorXi matching;
    Eigen::VectorXd effort;
    directional::principal_matching(VMeshWhole, FMeshWhole, EV, EF, FE, rawField, matching, effort);
    directional::effort_to_indices(VMeshWhole, FMeshWhole, EV, EF, effort, matching, N, singVertices, singIndices);
    directional::ParameterizationData pd;
    directional::cut_mesh_with_singularities(VMeshWhole, FMeshWhole, singVertices, pd.face2cut);
    directional::combing(VMeshWhole, FMeshWhole, EV, EF, FE, pd.face2cut, rawField, matching, combedField, combedMatching);
    // parametrizing
    std::cout << "[meshing] Setting up parameterization\n";
    directional::setup_parameterization(N, VMeshWhole, FMeshWhole, EV, EF, FE, combedMatching, singVertices, pd, VMeshCut, FMeshCut);
    std::cout << "[meshing] Solving parameterization\n";
    directional::parameterize(VMeshWhole, FMeshWhole, FE, combedField, lengthRatio, pd, VMeshCut, FMeshCut, isInteger, cutUV);
    std::cout << "[meshing] Done!\n";
}

void Meshing::cross_field_miq(const Eigen::MatrixXd& X1,
    const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
    const std::vector<std::vector<int>>& hard_edges,
    double gradient_size, int stiffen_iter,
    Eigen::MatrixXd& UV, Eigen::MatrixXi& FUV) {

    // Find the orthogonal field
    Eigen::MatrixXd B1, B2, B3;
    igl::local_basis(V, F, B1, B2, B3);
    Eigen::MatrixXd X2 = igl::rotate_vectors(X1, Eigen::VectorXd::Constant(1, igl::PI / 2), B1, B2);

    // Global parametrization
    igl::copyleft::comiso::miq(
        V,
        F,
        X1,
        X2,
        UV,
        FUV,
        gradient_size,
        5.0,   // stiffness, reserved but unused
        false, // direct round, default to the greedy rounding proposed in the MIQ paper
        stiffen_iter,
        5,     // local # iter of integer rounding
        true,  // do round = isInteger in Directional's parameterize()
        true,  // singularity round
        std::vector<int>(), // vertices to round, none other than singularities
        hard_edges
    );
}

// Solver data (needed for precomputation)
directional::PolyCurlReductionSolverData pcrdata;

int iter = 0;

void Meshing::init_curl(
    const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh, const int N,
    const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
    const Eigen::VectorXi& b, const Eigen::MatrixXd& bc, const Eigen::VectorXi& blevel,
    const Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
    Eigen::VectorXi& combedMatching, Eigen::VectorXd& combedEffort,
    Eigen::VectorXd& curl, Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
    Eigen::SparseMatrix<double>& AE2F, double& curlMax, double& curlMaxOrig) {

    Eigen::VectorXi matching;
    Eigen::VectorXd effort;
    directional::curl_matching(VMesh, FMesh, EV, EF, FE, rawField, matching, effort, curl);
    directional::effort_to_indices(VMesh, FMesh, EV, EF, effort, matching, N, singVertices, singIndices);
    directional::combing(VMesh, FMesh, EV, EF, FE, rawField, matching, combedField);
    directional::curl_matching(VMesh, FMesh, EV, EF, FE, combedField, combedMatching, combedEffort, curl);
    curlMaxOrig = curl.maxCoeff();
    curlMax = curlMaxOrig;
    std::cout << "curlMax original: " << curlMax << "\n";

    directional::polycurl_reduction_precompute(VMesh, FMesh, b, bc, blevel, rawField, pcrdata);
    iter = 0;

    // creating the AE2F operator
    std::vector<Eigen::Triplet<double>> AE2FTriplets;
    for (int i = 0; i < EF.rows(); i++) {
        if (EF(i,0) >= 0 && EF(i,0) < FMesh.rows()) AE2FTriplets.push_back(Eigen::Triplet<double>(EF(i,0), i, 1.0));
        if (EF(i,1) >= 0 && EF(i,1) < FMesh.rows()) AE2FTriplets.push_back(Eigen::Triplet<double>(EF(i,1), i, 1.0));
    }
    AE2F.resize(FMesh.rows(), EF.rows());
    AE2F.setFromTriplets(AE2FTriplets.begin(), AE2FTriplets.end());
}

// The set of parameters for calculating the curl-free fields
directional::polycurl_reduction_parameters params;

void Meshing::reduce_curl(
    const Eigen::MatrixXd& VMesh, const Eigen::MatrixXi& FMesh, const int N,
    const Eigen::MatrixXi& EV, const Eigen::MatrixXi& EF, const Eigen::MatrixXi& FE,
    Eigen::MatrixXd& rawField, Eigen::MatrixXd& combedField,
    Eigen::VectorXi& combedMatching, Eigen::VectorXd& combedEffort,
    Eigen::VectorXd& curl, Eigen::VectorXi& singVertices, Eigen::VectorXi& singIndices,
    double& curlMax) {
    
    Eigen::MatrixXd rawFieldNew, combedFieldNew;
    Eigen::VectorXi matching, combedMatchingNew;
    Eigen::VectorXd effort, combedEffortNew;
    Eigen::VectorXd curlNew;
    Eigen::VectorXi singVerticesNew, singIndicesNew;
    rawFieldNew = rawField;

    // do a batch of iterations
    std::cout << "--Improving Curl--\n";
    std::cout << "**** Batch " << iter << " ****\n";
    directional::polycurl_reduction_solve(pcrdata, params, rawFieldNew, iter == 0);
    ++iter;
    params.wSmooth *= params.redFactor_wsmooth;

    directional::curl_matching(VMesh, FMesh, EV, EF, FE, rawFieldNew, matching, effort, curlNew);
    directional::effort_to_indices(VMesh, FMesh, EV, EF, effort, matching, N, singVerticesNew, singIndicesNew);
    directional::combing(VMesh, FMesh, EV, EF, FE, rawFieldNew, matching, combedFieldNew);
    directional::curl_matching(VMesh, FMesh, EV, EF, FE, combedFieldNew, combedMatchingNew, combedEffortNew, curlNew);
    double curlMaxNew = curlNew.maxCoeff();
    if (curlMaxNew < curlMax) {
        std::cout << "curlMax optimized: " << curlMaxNew << "\n";
        rawField = rawFieldNew;
        combedField = combedFieldNew;
        combedMatching = combedMatchingNew;
        combedEffort = combedEffortNew;
        curl = curlNew;
        singVertices = singVerticesNew;
        singIndices = singIndicesNew;
    } else {
        std::cout << "curlMax failed to improve.\n";
    }
}

}
