#include "read_quad_mesh.h"

#include <igl/readOBJ.h>
#include <igl/remove_unreferenced.h>
#include <igl/planarize_quad_mesh.h>

namespace hlk {

    void read_quad_mesh(const std::string& obj_file, QuadMesh& Q, bool planarize) {

        std::vector<std::vector<double>> Vq, TCq, TC_DUMMY, Nq;
        std::vector<std::vector<int>> Fq, FTCq, FNq;

        igl::readOBJ(obj_file, Vq, TC_DUMMY, Nq, Fq, FTCq, FNq);

        // Remove non-quad faces
        Fq.erase(
            remove_if(
                Fq.begin(), Fq.end(),
                [](std::vector<int> f) {return f.size() != 4; }), Fq.end());

        // Initial Faces and Vertices, before removing unused vertices
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;

        V.resize(Vq.size(), 3);
        F.resize(Fq.size(), 4);

        for (int r = 0; r < Vq.size(); ++r) {
            for (int c = 0; c < 3; ++c) {
                V(r, c) = Vq[r][c];
            }
        }

        for (int r = 0; r < Fq.size(); ++r) {
            for (int c = 0; c < 4; ++c) {
                F(r, c) = Fq[r][c];
            }
        }

        // Copy vertex and face list to QuadMesh while removing unused vertices
        Eigen::VectorXi t1, t2; // Unused - needed to keep clang happy
        igl::remove_unreferenced(V, F, Q.V, Q.F_q, t1, t2);

        if (planarize) {
            Eigen::MatrixXd VP;
            igl::planarize_quad_mesh(Q.V, Q.F_q, 15, 0.005, VP);
            Q.V = VP;
        }

        Q.init();
    }
}
