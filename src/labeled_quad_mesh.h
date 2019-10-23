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

		std::vector<Eigen::VectorXi> slots;

		std::vector<Eigen::VectorXi> vertex_slots;
		std::vector<Eigen::VectorXi> dual_half_edge_slots;
		std::vector<Eigen::VectorXi> half_edge_slots;
		std::vector<Eigen::VectorXi> edge_slots;
		std::vector<Eigen::VectorXi> quad_slots;
		std::vector<Eigen::VectorXi> quadrant_slots; // q1-q4 for each face

		const Eigen::VectorXi& dual_half_edge_slot(int quad, int quad_side);
		const Eigen::VectorXi& half_edge_slot(int quad, int quad_side);
		//const Eigen::VectorXi& edge_slot(int u, int v);
		const Eigen::VectorXi& quadrant_slot(int quad, int quadrant); // Numbered by corner vertex
		const Eigen::VectorXi& quad_center_slot(int quad);

		void init();
		void init(double edge_width, double dual_edge_width);
		
	};

}