#include "labeled_quad_mesh.h"

#include <igl/EPS.h>
#include <igl/slice.h>
#include <igl/per_vertex_normals.h>
#include <igl/local_basis.h>

#include "glyph.h"

namespace hlk {

	void make_quadrant(
		const Eigen::MatrixXd& corners,
		int quadrant,
		Eigen::MatrixXd& quadrant_corners)
	{
		assert(corners.rows() == 4);
		assert(quadrant >= 0 && quadrant < 4);
		quadrant_corners.resize(4, 3);
		for (int i = 0; i < 4; ++i) {
			if (i != quadrant) {
				quadrant_corners.row(i) = (corners.row(i) + corners.row(quadrant)) / 2;
			}
			else {
				quadrant_corners.row(i) = corners.row(i);
			}
		}
	}

	void make_rect(
		const Eigen::MatrixXd& corners,
		const Eigen::MatrixXd& uvs,
		int layer,
		std::vector<Eigen::RowVector3d>& label_vertices,
		std::vector<Eigen::RowVector3i>& label_faces,
		std::vector<Eigen::RowVector2d>& label_uvs,
		Eigen::VectorXi& slot)
	{
		assert(corners.rows() == 4); // Input is actually a rectangle
		assert(uvs.rows() == 4 || uvs.rows() == 2);
		Eigen::MatrixXd uv(4, 2);
		if (uvs.rows() == 2) {
			uv <<
				uvs(0, 0), uvs(0, 1),
				uvs(1, 0), uvs(0, 1),
				uvs(1, 0), uvs(1, 1),
				uvs(0, 0), uvs(1, 1);
		}
		else {
			uv = uvs;
		}

		Eigen::RowVector3d v1 = corners.row(1) - corners.row(0);
		Eigen::RowVector3d v2 = corners.row(3) - corners.row(0);
		Eigen::RowVector3d N = v1.cross(v2).normalized();
		Eigen::RowVector3d offset = layer * ((double)igl::FLOAT_EPS) * 5000 * N;

		int s = label_vertices.size();
		int slot_i = slot.rows();
		slot.conservativeResize(slot_i + 4);
		for (int i = 0; i < 4; ++i) {
			label_vertices.push_back(corners.row(i) + offset);
			label_uvs.push_back(uv.row(i));
			slot[slot_i + i] = s + i;
		}
		label_faces.push_back(Eigen::RowVector3i(s, s + 1, s + 2));
		label_faces.push_back(Eigen::RowVector3i(s, s + 2, s + 3));

	}

	const Eigen::VectorXi& LabeledQuadMesh::dual_half_edge_slot(int quad, int quad_side)
	{
		return dual_half_edge_slots[4 * quad + quad_side];
	}

	const Eigen::VectorXi& LabeledQuadMesh::half_edge_slot(int quad, int quad_side)
	{
		return half_edge_slots[4 * quad + quad_side];
		// TODO: insert return statement here
	}

	const Eigen::VectorXi& LabeledQuadMesh::quadrant_slot(int quad, int quadrant)
	{
		return quadrant_slots[4 * quad + quadrant];
	}

	const Eigen::VectorXi& LabeledQuadMesh::quad_center_slot(int quad)
	{
		return vertex_slots[quad];
		//return vertex_slots[n + quad];
	}

	void LabeledQuadMesh::init()
	{
		init(0.07, 0.2);
	}

	void LabeledQuadMesh::init(double edge_width, double dual_edge_width)
	{
		QuadMesh::init();

		// Make the slots
		std::vector<Eigen::RowVector3d> label_vertices;
		std::vector<Eigen::RowVector3i> label_faces;
		std::vector<Eigen::RowVector2d> label_uvs;

		std::vector<Eigen::MatrixXd> side_corners; // temporary storage of half-edge-label-coordinates
		/*
		Layers:

		Highlights (same as below but floating above)

		Vertices
		Face Centers
		Dual-Half-Edges (sides)
		Half-Edges (sides)
		Edges (only true edges)
		Face Quadrants
		Face
		*/

		Eigen::MatrixXd uv(2, 2), bottom_half_uv(2, 2), top_half_uv(2,2);
		uv <<
			0, 0,
			1, 1;
		bottom_half_uv <<
			0, 0,
			1, 0.5;
		top_half_uv <<
			0, 0.5,
			1, 1;

		Eigen::MatrixXd VN;
		igl::per_vertex_normals(V, F_t, VN);

		/*
		// Quad Vertex slots
		for (int i = 0; i < n; ++i) {
			Eigen::RowVector3d vpos = V.row(i);
			Eigen::RowVector3d normal = VN.row(i);
			Eigen::RowVector3d b(8.2, 7.345, 3.24);
			b.normalize();
			Eigen::RowVector3d x = normal.cross(b).normalized();
			Eigen::RowVector3d y = normal.cross(x).normalized();
			Eigen::MatrixXd vertex_corners(4, 3);
			vertex_corners <<
				vpos + x * dual_edge_width / 2,
				vpos + y * dual_edge_width / 2,
				vpos - x * dual_edge_width / 2,
				vpos - y * dual_edge_width / 2;
			Eigen::VectorXi vertex_slot;

			make_rect(vertex_corners, uv, 18, label_vertices, label_faces, label_uvs, vertex_slot);
			slots.push_back(vertex_slot);
			vertex_slots.push_back(vertex_slot);
		}
		*/

		// Per Face Slots
		for (int i = 0; i < F_q.rows(); ++i) {
			Eigen::MatrixXd corners;
			Eigen::VectorXi face = F_q.row(i);
			igl::slice(V, face, 1, corners);
			Eigen::VectorXi slot;
			make_rect(corners, uv, 1, label_vertices, label_faces, label_uvs, slot);
			slots.push_back(slot);
			quad_slots.push_back(slot);

			// Quadrants!
			for (int j = 0; j < 4; ++j) {
				Eigen::MatrixXd quadrant_corners;
				Eigen::VectorXi qslot;
				make_quadrant(corners, j, quadrant_corners);
				make_rect(quadrant_corners, uv, 2, label_vertices, label_faces, label_uvs, qslot);
				slots.push_back(qslot);
				quadrant_slots.push_back(qslot);
			}

			// Center
			double a = dual_edge_width / 2;
			Eigen::MatrixXd center_corners(4,3);
			Eigen::VectorXi center_slot;
			Eigen::RowVector3d center_pos = (corners.row(0) + corners.row(2)) / 2;
			Eigen::RowVector3d d2 = corners.row(1) - corners.row(3);
			Eigen::RowVector3d d1 = corners.row(2) - corners.row(0);
			center_corners <<
				center_pos + a * d2,
				center_pos + a * d1,
				center_pos - a * d2,
				center_pos - a * d1;
				
			make_rect(center_corners, uv, 14, label_vertices, label_faces, label_uvs, center_slot);
			slots.push_back(center_slot);
			vertex_slots.push_back(center_slot); /// Face centers are on tri-vertices

			// Per Side Slots
			for (int j = 0; j < 4; ++j) {
				int side = 4 * i + j;

				// Half-edges
				Eigen::MatrixXd half_edge_corners(4,3);
				half_edge_corners <<
					corners.row(j),
					corners.row((j + 1) % 4),
					corners.row((j + 1) % 4) + (corners.row((j + 2) % 4) - corners.row((j + 1) % 4))* edge_width,
					corners.row(j) + (corners.row((j + 3) % 4) - corners.row(j))* edge_width;
					

				side_corners.push_back(half_edge_corners);

				Eigen::VectorXi half_edge_slot, dual_half_edge_slot;

				make_rect(half_edge_corners, uv, 3+j, label_vertices, label_faces, label_uvs, half_edge_slot);
				slots.push_back(half_edge_slot);
				half_edge_slots.push_back(half_edge_slot);

				// Dual-Half-edges
				double d = 0.5 - dual_edge_width / 2;
				Eigen::RowVector3d bottom = corners.row(j) + (corners.row((j + 3) % 4) - corners.row(j)) * d;
				Eigen::RowVector3d top = corners.row((j + 1) % 4) + (corners.row((j + 2) % 4) - corners.row((j + 1) % 4)) * d;

				Eigen::MatrixXd dual_edge_corners(4,3);
				dual_edge_corners <<
					d * top + (1 - d)* bottom,
					(1 - d)* corners.row(j) + d * corners.row((j + 1) % 4),
					d* corners.row(j) + (1 - d)* corners.row((j + 1) % 4),
					d* bottom + (1 - d)* top;
					
					

				make_rect(dual_edge_corners, uv, 10+j, label_vertices, label_faces, label_uvs, dual_half_edge_slot);
				slots.push_back(dual_half_edge_slot);
				dual_half_edge_slots.push_back(dual_half_edge_slot);
			}
		}

		// Edge Based
		for (int i = 0; i < e; ++i) {
			Eigen::RowVector2i edge_sides = edges_to_sides.row(i);
			Eigen::MatrixXd corners1 = side_corners[edge_sides[0]];
			Eigen::MatrixXd corners2 = side_corners[edge_sides[1]];
			Eigen::VectorXi edge_slot;
			Eigen::MatrixXd bottom_half(4,3);
	
			// One half of the combined edges needs to be rotated by 180 deg. to match orientations
			// arbitrarly choose the bottom half since there is no prescribed ordering
			bottom_half <<
				corners1.row(2),
				corners1.row(3),
				corners1.row(0),
				corners1.row(1);

			make_rect(bottom_half, bottom_half_uv, 8, label_vertices, label_faces, label_uvs, edge_slot);
			make_rect(corners2, top_half_uv, 8, label_vertices, label_faces, label_uvs, edge_slot);
			slots.push_back(edge_slot);
			edge_slots.push_back(edge_slot);
		}


		LV.resize(label_vertices.size(), 3);
		LF.resize(label_faces.size(), 3);
		UV.resize(label_uvs.size(), 2);
		LUV.resize(label_uvs.size(), 2);
		C.resize(label_vertices.size(), 4);

		for (int i = 0; i < label_vertices.size(); ++i) {
			LV.row(i) = label_vertices[i];
			C.row(i) = Eigen::RowVector4d(1.0, 1.0, 1.0, 0.0);
		}
		for (int i = 0; i < label_faces.size(); ++i) {
			LF.row(i) = label_faces[i];
		}
		for (int i = 0; i < label_uvs.size(); ++i) {
			UV.row(i) = label_uvs[i];
			LUV.row(i) = label_uvs[i];
		}
	}

	void LabeledQuadMesh::set_glyph(const Eigen::VectorXi& slot, const Eigen::MatrixXd& glyph, const Eigen::RowVector4d& color)
	{
		hlk::set_glyph(LUV, slot, glyph, color, UV, C);
	}

}