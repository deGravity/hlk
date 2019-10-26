#include "coarse_knit_mesh.h"

namespace hlk {


	void CoarseKnitMesh::init()
	{
		LabeledQuadMesh::init();
		
		// Create Topological Entities
		// TODO - If optimization is buggy, switch these back to smart pointers,
		// the error is probably due to copy constructor shenanigans
		// Hopefully there won't be a problem since all z3 variables are stored
		// as shared pointers.
		for (int q = 0; q < m; ++q) {
			
			for (int j = 0; j < 4; ++j) {
				sides.emplace_back(geometry_optimizer, topology_optimizer, 4 * q + j, this);
			}

			// It is important that these come second since they rely on
			// the sides to compute orientation!
			quads.emplace_back(geometry_optimizer, topology_optimizer, q, this);
			
		}
		for (int i = 0; i < e; ++i) {
			edges.emplace_back(geometry_optimizer, topology_optimizer, i, this);
		}

	}

	const char* nth_label(std::string label, int n) {
		return (label + "_" + std::to_string(n)).c_str();
	}

	CoarseKnitEdge::CoarseKnitEdge(Optimizer & geo_opt, Optimizer & topo_opt, double len, int i, CoarseKnitMesh * m)
	{
		is_seam = geo_opt.get_bool_prop(nth_label("is_seam", i));
		index = i;
		mesh = m;
	}
	CoarseKnitQuad::CoarseKnitQuad(Optimizer & geo_opt, Optimizer & topo_opt, int i, CoarseKnitMesh * m)
	{
		shaping_distribution = DISTRIBUTED;
		short_row_distribution = NONE;
		index = i;
		mesh = m;
		// TODO - I think we're not using this anymore...
		time = geo_opt.get_int_prop(nth_label("time", i));

		// TODO - Handle textures more gracefully
		texture.resize(1, 1);
		texture(0, 0) = false;
		texture_color = Eigen::Vector3d(1, 1, 1);

		// Setup Handedness

		auto& left = m->sides[m->nth_side(i, 3)];
		auto& center = m->sides[m->nth_side(i, 0)];
		auto& right = m->sides[m->nth_side(i, 1)];

		auto& ll = left.is_loop;
		auto& ld = left.is_out;
		auto& cl = center.is_loop;
		auto& cd = center.is_out;
		auto& rl = right.is_loop;
		auto& rd = right.is_out;

		auto left_switches = cl != ll;
		auto right_switches = cl != rl;
		auto right_ccw = cl != (cd == rd);
		auto left_cw = cl != (cd == ld);
		auto is_ccw = (right_switches && right_ccw) || (left_switches && !left_cw);

		handedness = std::make_shared<z3::expr>(is_ccw);
	}
	CoarseKnitSide::CoarseKnitSide(Optimizer & geo_opt, Optimizer & topo_opt, int i, CoarseKnitMesh * m)
	{
		is_loop = geo_opt.get_bool_prop(nth_label("is_loop", i));
	}
}