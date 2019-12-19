#pragma once

#include "quad_mesh.h"
#include <vector>

namespace hlk {

    struct LabeledQuadMesh : QuadMesh {

        Eigen::MatrixXd LV; // Label Vertices
        Eigen::MatrixXi LF; // Label Faces
        Eigen::MatrixXd LUV; // Label Texture Coords

        Eigen::MatrixXd UV; // Actual Texture Coords
        Eigen::MatrixXd C; // Label Colors

        std::vector<Eigen::RowVector3d> on_mesh_vertices;
        std::vector<int> vertex_layers;

        std::vector<Eigen::VectorXi> slots;

        std::vector<Eigen::VectorXi> vertex_slots;
        std::vector<Eigen::VectorXi> dual_half_edge_slots;
        std::vector<Eigen::VectorXi> half_edge_slots;
        std::vector<Eigen::VectorXi> edge_slots;
        std::vector<Eigen::VectorXi> quad_slots;
        std::vector<Eigen::VectorXi> quadrant_slots; // labeled by vertex index for each face

        const Eigen::VectorXi& dual_half_edge_slot(int quad, int quad_side);
        const Eigen::VectorXi& half_edge_slot(int quad, int quad_side);
        //const Eigen::VectorXi& edge_slot(int u, int v);
        const Eigen::VectorXi& quadrant_slot(int quad, int quadrant); // Numbered by corner vertex
        const Eigen::VectorXi& quad_center_slot(int quad);

        virtual void init();
        virtual void init(double edge_width, double dual_edge_width);

        void set_glyph(const Eigen::VectorXi& slot, const Eigen::MatrixXd& glyph, const Eigen::RowVector4d& color);
        
        void remap_to_surface(Eigen::MatrixXd& SV, Eigen::MatrixXi& SF, int subdivision_iters = 4);

        void make_rect(
            const Eigen::MatrixXd& corners,
            const Eigen::MatrixXd& uvs,
            int layer,
            std::vector<Eigen::RowVector3d>& label_vertices,
            std::vector<Eigen::RowVector3i>& label_faces,
            std::vector<Eigen::RowVector2d>& label_uvs,
            Eigen::VectorXi& slot);

    };

}
