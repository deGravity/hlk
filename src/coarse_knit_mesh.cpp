#include "coarse_knit_mesh.h"

#include <algorithm>

#include <igl/parula.h>
#include <igl/jet.h>

#include "glyphs.h"
#include "glyph.h"

namespace hlk {

	std::string nth_label(std::string label, int n) {
		return (label + "_" + std::to_string(n));
	}

	void CoarseKnitMesh::update_textures()
	{
		// Make Everything Invisible to start
		for (auto& slot : slots) {
			set_glyph(slot, glyphs::NONE, color::INVISIBLE);
		}

		for (int i = 0; i < 4 * m; ++i) {
			if (is_boundary_side[i]) {
				set_glyph(half_edge_slots[i], glyphs::THIN_SOLID_LINE, color::GREY);
			}
		}

		for (auto& slot : edge_slots) {
			set_glyph(slot, glyphs::THIN_SOLID_LINE, color::GREY);
		}


		for (auto& edge : edges) {
			edge.update_texture();
		}
		for (auto& side : sides) {
			side.update_texture();
		}

		// Get a time scale
		int num_sources = 0;
		int num_sinks = 0;
		for (auto& quad : quads) {
			if (quad.time->val < min_time) min_time = quad.time->val;
			if (quad.time->val > max_time) max_time = quad.time->val;

			if (sides[quad.index].is_loop->val && sides[quad.index + 1].is_loop->val && sides[quad.index + 2].is_loop->val && sides[quad.index + 3].is_loop->val) {

			}
		}

		for (auto& quad : quads) {
			quad.update_texture();
		}
	}

	void CoarseKnitMesh::init()
	{
		LabeledQuadMesh::init();

		for (int q = 0; q < m; ++q) {
			
			for (int j = 0; j < 4; ++j) {
				sides.emplace_back(geometry_optimizer, topology_optimizer, 4 * q + j, this);
			}
			quads.emplace_back(geometry_optimizer, topology_optimizer, q, this);
			
		}
		for (int i = 0; i < e; ++i) {
			edges.emplace_back(geometry_optimizer, topology_optimizer, i, this);
		}

		vertex_in_seam.resize(n, false);

		/*	
		From Motorcycle Graphs Paper:

		Thus, we may often find a smaller partition than the mo-torcycle graph itself by a process 
		in which we build up the partition by adding a single path at a time, at each step start-ing 
		from an extraordinary vertex and extending a path from it until it hits either another 
		extraordinary vertex or an ordinaryvertex that has previously been included in one of the 
		paths. In this process, we should give priority first to paths that ex-tend from one 
		extraordinary vertex to another, because these paths cannot cause us to add any additional
		vertices to our partition. Secondly, we should prefer paths the initial edge ofwhich is an
		even number of positions from some other edge around the same extraordinary vertex, in order 
		to use as few paths emanating from that vertex as possible. Once no two consecutive edges at 
		an extraordinary vertex remain unused, the partition process may terminate with a valid
		partition. The partition in Figure5, for instance, may be constructed by a process of this
		type, and is significantly simpler thanthe motorcycle graph partition of the same mesh 
		in Figure7.
		*/

		// Find singularity connecting edges and add them to seams
		// TODO - We can probably split these at intersections
		// is it worth doing?
		for (int sv : singular_vertices) {
			for (int side : out_sides(sv)) {
				auto separatrix = side_loop(side);
				if (is_singularity[side_v(separatrix.back())]) {
					if (!is_boundary_side[side]) {
						int sep_seam = edges[sides_to_edges[side]].seam;
						bool new_seam = false;
						if (sep_seam == -1) {
							new_seam = true;
							sep_seam = seams.size();
							seams.emplace_back(geometry_optimizer.get_bool_prop(nth_label("seam", sep_seam)));
						}
						std::vector<int> s_edges;
						for (int seam_side : separatrix) {
							int e = sides_to_edges[seam_side];
							edges[e].seam = sep_seam;
							s_edges.push_back(e);
							vertex_in_seam[side_u(seam_side)] = true;
							vertex_in_seam[side_v(seam_side)] = true;
						}
						if (new_seam) {
							seam_edges.push_back(s_edges);
						}
					}
				}
			}
		}

		

		// TODO - Bug with the solver! It's probably due to not using shared pointers
		// try changing this next.
		update_textures();

		std::cout << "Mesh Loaded, Optimizing Topology" << std::endl;
		// Solve the SMT problem and update the textures
		if (!optimize_geometry()) {
			std::cout << "Unable to initialize" << std::endl;
		}

	}

	

	CoarseKnitEdge::CoarseKnitEdge(Optimizer & geo_opt, Optimizer & topo_opt, int i, CoarseKnitMesh * m)
	{
		seam = -1;
		index = i;
		mesh = m;
	}
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitEdge::get_constraints()
	{
		std::vector<std::pair<z3::expr, std::string>> constraints;
		int side_a_idx = mesh->edges_to_sides(index, 0);
		int side_b_idx = mesh->edges_to_sides(index, 1);
		auto& side_a = mesh->sides[side_a_idx];
		auto& side_b = mesh->sides[side_b_idx];
		int quad_a_idx = side_a_idx / 4;
		int quad_b_idx = side_b_idx / 4;
		auto& time_a = mesh->quads[quad_a_idx].time->var;
		auto& time_b = mesh->quads[quad_b_idx].time->var;

		std::string edge_consistency_name = "edge_consistency_" + std::to_string(index);
		std::string time_alignment_name = "time_alignment_" + std::to_string(index);


		auto edge_consistency = (side_a.is_loop == side_b.is_loop) && (side_a.is_out != side_b.is_out);
		// TODO - Add more IntProp and BoolProp overloads to simplify time alignment
		
		auto time_alignment =
			(side_a.is_loop->var && side_a.is_out->var && (time_a < time_b)) || 
			(side_a.is_loop->var && !side_a.is_out->var && (time_b < time_a)) ||
			(!side_a.is_loop->var && (time_a == time_b));

		if (seam >= 0) {
			constraints.push_back(std::make_pair(mesh->seams[seam]->var || edge_consistency, edge_consistency_name));
			constraints.push_back(std::make_pair(mesh->seams[seam]->var || time_alignment, time_alignment_name));
		}
		else {
			constraints.push_back(std::make_pair(edge_consistency, edge_consistency_name));
			constraints.push_back(std::make_pair(time_alignment, time_alignment_name));
		}

		return constraints;
	}
	void CoarseKnitEdge::update_texture()
	{
		if (seam >= 0) {
			if (mesh->seams[seam]->val) {
				if (mesh->seams[seam]->is_fixed) {
					mesh->set_glyph(mesh->edge_slots[index], glyphs::SOLID_LINE, color::BLUE);
				}
				else {
					mesh->set_glyph(mesh->edge_slots[index], glyphs::DASHED_LINE, color::BLUE);
				}
			}
			else {
				if (mesh->seams[seam]->is_fixed) {
					mesh->set_glyph(mesh->edge_slots[index], glyphs::SOLID_LINE, color::GREY);
				}
				else {
					mesh->set_glyph(mesh->edge_slots[index], glyphs::DASHED_LINE, color::GREY);
				}
			}

		}
	}
	CoarseKnitQuad::CoarseKnitQuad(Optimizer & geo_opt, Optimizer & topo_opt, int i, CoarseKnitMesh * m)
	{
		shaping_distribution = DISTRIBUTED;
		short_row_distribution = NONE;
		index = i;
		mesh = m;
		time = geo_opt.get_int_prop(nth_label("time", i));
	}
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitQuad::get_constraints()
	{
		int q = this->index;
		auto& s0_o = mesh->sides[q].is_out->var;
		auto& s0_l = mesh->sides[q].is_loop->var;
		auto& s1_o = mesh->sides[q+1].is_out->var;
		auto& s1_l = mesh->sides[q+1].is_loop->var;
		auto& s2_o = mesh->sides[q+2].is_out->var;
		auto& s2_l = mesh->sides[q+2].is_loop->var;
		auto& s3_o = mesh->sides[q+3].is_out->var;
		auto& s3_l = mesh->sides[q+3].is_loop->var;

		// vertical skips are bad
		auto no_skip_0 = !s0_l || // X
			(s0_l != s2_l) || // X
			(s0_o != s2_o) || // O
			((s1_l == s0_l && s1_o == s0_o) || //
			(s3_l == s0_l && s3_o == s0_o)); //

		auto no_skip_1 = !s1_l || // X
			(s1_l != s3_l) || // X
			(s1_o != s3_o) || // 
			((s2_l == s1_l && s2_o == s1_o) || //
			(s0_l == s1_l && s0_o == s1_o)); //

		auto sink_source = s0_l && s1_l && s2_l && s3_l && (s0_o == s1_o) && (s1_o == s2_o) && (s2_o == s3_o) && (s3_o == s0_o);

		/*
		auto opp_loop = [&](int a, int b) {
			return mesh->sides[q + a].is_out != mesh->sides[b].is_out && mesh->sides[a].is_loop && mesh->sides[b].is_loop;
		};
		*/

		//auto criss_cross = opp_loop(0, 2) && opp_loop(1, 3);

		auto criss_cross = s0_l && s1_l && s2_l && s3_l && (s0_o != s2_o) && (s1_o != s3_o);

		std::vector<std::pair<z3::expr, std::string>> constraints;
		
		constraints.push_back(std::make_pair(
			no_skip_0,
			"no_skip_0_" + std::to_string(q)
		));

		constraints.push_back(std::make_pair(
			no_skip_1,
			"no_skip_1_" + std::to_string(q)
		));

		
		constraints.push_back(std::make_pair(
			!criss_cross,
			"no_criss_cross_" + std::to_string(q)
		));
		
		constraints.push_back(std::make_pair(
			!sink_source,
			"no_sink_source_" + std::to_string(q)
		));

		return constraints;
	}
	void CoarseKnitQuad::update_texture()
	{
		int min_t = mesh->min_time;
		int max_t = mesh->max_time;
		double color_v = (double)(time->val - min_t) / (max_t - min_t);
		double r, g, b;
		igl::parula(color_v, r, g, b);
		Eigen::RowVector4d color(r, g, b, 1.0);
		auto slot = mesh->quad_slots[index];
		mesh->set_glyph(slot, glyphs::SOLID_LINE, color);
		// TODO - Print Glyphs for inc/dec type
	}
	CoarseKnitSide::CoarseKnitSide(Optimizer & geo_opt, Optimizer & topo_opt, int i, CoarseKnitMesh * m)
	{
		is_loop = geo_opt.get_bool_prop(nth_label("is_loop", i));
		is_out = geo_opt.get_bool_prop(nth_label("is_out", i));
		index = i;
		mesh = m;
	}

	std::vector<std::pair<z3::expr,std::string>> CoarseKnitSide::get_constraints()
	{
		std::vector<std::pair<z3::expr, std::string>> constraints;
		int u = mesh->side_u(index);
		bool is_border = mesh->is_border_vertex[u];
		int valence = mesh->valence[u];
		// For our purposes, valences of multiples of 4 (3 on borders)
		// do not count as singularities, so calculate this explicitly
		bool is_regular = (is_border && (valence % 3 == 0)) || (valence % 4 == 0);
		int prev_idx = mesh->prev_side(index);
		auto& prev_is_loop = mesh->sides[prev_idx].is_loop;
		auto& prev_is_out = mesh->sides[prev_idx].is_out;
		if (false){//is_regular) {
			// Handedness order is loop_in -> yarn_out -> loop_out -> yarn_in
			// The orientation (in/out) switches after loops and stays the
			// same after yarns, while the direction (loop/yarn) always switches.
			// If both conditions are true, then the corner has proper handedness
			auto orientation_check = (prev_is_out == is_out) != prev_is_loop;
			auto direction_check = prev_is_loop != is_loop;

			constraints.push_back(
				std::make_pair(
					orientation_check && direction_check,
					"corner_" + std::to_string(index)
				)
			);

			//constraints.push_back(orientation_check && direction_check);

		}
		else {
			// Constrain that the order at a singular corner can't go "backwards"
			// by 1. This looks like the the above constraint, except that loop/yarn are
			// switched (going "backward") and the entire term is negated (prohibit that path)

			// TODO - Constrain further - only allow doubling of loops.

			auto orientation_check = (prev_is_out == is_out) == prev_is_loop; // == -> !=
			auto direction_check = prev_is_loop != is_loop; // Don't need to add ! to both sides since it would cancel

			auto no_yarn_double = !(!prev_is_loop->var && !is_loop->var);

			constraints.push_back(
				std::make_pair(
					!(orientation_check && direction_check) && // Negate full expression
					no_yarn_double,
					"singular_corner_" + std::to_string(index)
				)
			);

		}
		return constraints;
	}
	void CoarseKnitSide::update_texture()
	{
		auto& arrow = is_loop->is_fixed ? glyphs::SOLID_ARROW : glyphs::DASHED_ARROW;
		auto& line = is_loop->is_fixed ? glyphs::THIN_SOLID_LINE : glyphs::THIN_DASHED_LINE;
		auto& s_color = is_loop->val ? color::ORANGE : color::GREEN;
		auto& symbol = is_out->val ? line : arrow;
		mesh->set_glyph(mesh->dual_half_edge_slots[index], symbol, s_color);
	}
	bool CoarseKnitMesh::optimize_geometry()
	{
		geometry_optimizer.push();

std::vector<z3::expr> seam_costs;
int i = 0;
std::cout << "Num possible seams = " << seams.size();
for (auto& seam : seams) {
	std::string cost_name = "seam_cost_" + std::to_string(i);
	z3::expr s_cost = geometry_optimizer.context.int_const(cost_name.c_str());
	seam_costs.push_back(s_cost);
	geometry_optimizer.add_constraint((seam->var && s_cost == 1) || (!seam->var && s_cost == 0));
	++i;
}


for (auto& edge : edges) {
	for (auto constraint : edge.get_constraints()) {
		geometry_optimizer.add_constraint(constraint.first, constraint.second);
	}
}
for (auto& side : sides) {
	for (auto constraint : side.get_constraints()) {
		geometry_optimizer.add_constraint(constraint.first, constraint.second);
	}
}

for (auto& quad : quads) {
	for (auto constraint : quad.get_constraints()) {
		geometry_optimizer.add_constraint(constraint.first, constraint.second);
	}
}



z3::expr cost = geometry_optimizer.context.int_const("cst");
if (seam_costs.size() > 0) {
	cost = seam_costs[0];
	for (int i = 1; i < seam_costs.size(); ++i) {
		cost = cost + seam_costs[i];
	}
}

auto result = seam_costs.size() > 0 ? geometry_optimizer.minimize(cost, 15) : geometry_optimizer.solve();

if (result.has_result) {
	geometry_optimizer.update_all_props(*result.result_model);
	update_textures();
}
else {
	// TODO - Get Information from the UNSAT core
	std::cout << result.unsat_core << std::endl;
}

geometry_optimizer.pop();
return result.has_result;
	}
	void CoarseKnitMesh::copy_shaping(int origin_side, int dest_side)
	{
		auto& origin_quad = quads[origin_side / 4];
		auto& dest_quad = quads[dest_side / 4];

		dest_quad.shaping_distribution = origin_quad.shaping_distribution;
		dest_quad.short_row_distribution = origin_quad.short_row_distribution;
	}
	void CoarseKnitMesh::set_texture(int side, int texture)
	{
		quads[side / 4].texture_id = texture;
	}
	void CoarseKnitMesh::seam_off(int side)
	{
		int edge_id = sides_to_edges[side];
		if (edge_id >= 0) {
			int seam_id = edges[edge_id].seam;
			if (seam_id >= 0) {
				seams[seam_id]->set(false);
			}
		}
	}
	void CoarseKnitMesh::seam_on(int side)
	{
		int edge_id = sides_to_edges[side];
		if (edge_id >= 0) {
			int seam_id = edges[edge_id].seam;
			if (seam_id >= 0) {
				seams[seam_id]->set(true);
			}
		}
	}
	void CoarseKnitMesh::toggle_seam(int side)
	{
		int edge_id = sides_to_edges[side];
		if (edge_id >= 0) {
			int seam_id = edges[edge_id].seam;
			if (seam_id >= 0) {
				seams[seam_id]->set(!seams[seam_id]->val);
			}
		}
	}
	void CoarseKnitMesh::add_seam(std::vector<int> seam_sides)
	{
		for (auto side : seam_sides) {
			assert(sides_to_edges[side] >= 0);
			assert(edges[sides_to_edges[side]].seam < 0);
		}
		int seam_id = seams.size();
		seams.emplace_back(geometry_optimizer.get_bool_prop(nth_label("seam", seam_id)));
		std::vector<int> new_seam;
		for (auto side : seam_sides) {
			int e = sides_to_edges[side];
			edges[e].seam = seam_id;
			new_seam.push_back(e);
		}
		seam_edges.push_back(new_seam);

	}
	void CoarseKnitMesh::split_seams(int vertex_a, int vertex_b)
	{
		// Since split_seams doesn't do anything if a seam doesn't
		// contain a vertex, try to split all the seams
		std::vector<int> split_vtcs{ vertex_a, vertex_b };
		for (int i = 0; i < seams.size(); ++i) {
			split_seam(i, split_vtcs);
		}
	}
	void CoarseKnitMesh::split_seam(int seam, std::vector<int> vertices)
	{
		auto& s_edges = seam_edges[seam];
		
		if (s_edges.size() <= 1) return; // Single edge seams cannot be split

		auto edge_has_vertex = [&](int v, int edge_id)->bool {
			return side_u(edges_to_sides(edge_id, 0)) == v ||
				side_v(edges_to_sides(edge_id, 0)) == v;
		};

		std::vector<int> split_points;
		for (int v : vertices) {
			int i = 0;
			for (i = 1; i < s_edges.size(); ++i) {
				if (edge_has_vertex(v, s_edges[i])) break;
			}
			if (i < s_edges.size()) {
				if (edge_has_vertex(v, s_edges[(i + 1) % s_edges.size()])) {
					i = (i + 1) % s_edges.size();
				}
				split_points.push_back(i);
			}
		}

		if (split_points.size() == 0) return; // Nothing to do if vertices aren't in the seam

		// Sort and deduplicate split points list
		std::sort(split_points.begin(), split_points.end());
		split_points.erase(std::unique(split_points.begin(), split_points.end()), split_points.end());

		// Check if the seam is a loop: if so, we need a seam between the last and first split_point, otherwise
		// we need seams extending to the endpoints of the original seam
		bool is_circular = false;
		auto front = s_edges.front();
		auto back = s_edges.back();
		int front_side = edges_to_sides(front, 0);
		if (edge_has_vertex(side_u(front_side), back) || edge_has_vertex(side_v(front_side), back)) {
			is_circular = true;
		}

		// Find all the new seams
		std::vector<std::vector<int>> new_seams;
		if (is_circular) {
			for (int i = 0; i < split_points.size(); ++i) {
				int start = split_points[i];
				int end = split_points[(i + 1) % split_points.size()];
				std::vector<int> new_seam;
				for (int e = start; e != end; ++e) {
					new_seam.push_back(s_edges[e]);
				}
				new_seams.push_back(new_seam);
			}
		}
		else {
			if (split_points[0] > 0) {
				std::vector<int> new_seam;
				for (int i = 0; i < split_points[0]; ++i) {
					new_seam.push_back(s_edges[i]);
				}
				new_seams.push_back(new_seam);
			}
			for (int i = 0; i < split_points.size() - 1; ++i) {
				int start = split_points[i];
				int end = split_points[(i + 1) % split_points.size()];
				std::vector<int> new_seam;
				for (int e = start; e != end; ++e) {
					new_seam.push_back(s_edges[e]);
				}
				new_seams.push_back(new_seam);
			}
			if (split_points.back() < s_edges.size() - 1) {
				std::vector<int> new_seam;
				for (int e = split_points.back(); e < s_edges.size(); ++e) {
					new_seam.push_back(s_edges[e]);
				}
				new_seams.push_back(new_seam);
			}
		}

		// If there is only one new seam, there's nothing to do
		if (new_seams.size() <= 1) return;

		// Swap the old seam for the first new seam, then add all of the new ones
		seam_edges[seam] = new_seams[0];
		for (int i = 1; i < new_seams.size(); ++i) {
			int new_seam_id = seams.size();
			seams.emplace_back(geometry_optimizer.get_bool_prop(nth_label("seam", new_seam_id)));
			seam_edges.push_back(new_seams[i]);
			for (int e : new_seams[i]) {
				edges[e].seam = new_seam_id;
			}
		}


	}
	void CoarseKnitMesh::toggle_orientation(int side)
	{
		sides[side].is_out->set(!sides[side].is_out->val);
		sides[side].is_loop->is_fixed = true;
	}
	void CoarseKnitMesh::toggle_direction(int side)
	{
		sides[side].is_loop->set(!sides[side].is_loop->val);
		sides[side].is_out->is_fixed = true;
	}
	void CoarseKnitMesh::paint_direction(int side_out, int side_in, KnitDirection dir)
	{
		sides[side_out].is_out->set(true);
		sides[side_in].is_out->set(false);
		bool is_loop = dir == LOOP;
		sides[side_out].is_loop->set(is_loop);
		sides[side_in].is_loop->set(is_loop);
		sides[side_in].update_texture();
		sides[side_out].update_texture();
	}
	void CoarseKnitMesh::erase_seam(int side)
	{
		int edge_id = sides_to_edges[side];
		if (edge_id >= 0) {
			int seam_id = edges[edge_id].seam;
			if (seam_id >= 0) {
				seams[seam_id]->set(false, false);
			}
		}
	}
	void CoarseKnitMesh::erase_orientation(int side)
	{
		sides[side].is_loop->is_fixed = false;
		sides[side].is_out->is_fixed = false;
		sides[side].update_texture();
	}
	void CoarseKnitMesh::erase_textue(int side)
	{
		quads[side / 4].texture_id = -1;
	}
	void CoarseKnitMesh::erase_shaping(int side)
	{
		quads[side / 4].shaping_distribution = NONE;
		quads[side / 4].short_row_distribution = NONE;
	}
}