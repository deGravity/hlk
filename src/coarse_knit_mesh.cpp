#include "coarse_knit_mesh.h"
#include "serialization.h"

#include <algorithm>

#include <igl/parula.h>
#include <igl/jet.h>
#include <igl/barycenter.h>

#include "glyphs.h"
#include "glyph.h"

#include "texture.h"

#include "disjointset.h"

#include "symmetrizer.h"

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
		/*
		int num_sources = 0;
		int num_sinks = 0;
		for (auto& quad : quads) {
			if (quad.time->val < min_time) min_time = quad.time->val;
			if (quad.time->val > max_time) max_time = quad.time->val;

			if (sides[4*quad.index].is_loop->val && sides[4*quad.index + 1].is_loop->val && sides[4*quad.index + 2].is_loop->val && sides[4*quad.index + 3].is_loop->val) {

			}
		}
		*/

		for (auto& quad : quads) {
			quad.update_texture();
		}

		for (auto sv : singular_vertices) {
			set_glyph(vertex_slots[sv], glyphs::CROSS, color::RED);
		}

		for (int i = 0; i < vertex_is_special.size(); ++i) {
			if (vertex_is_special[i]) {
				set_glyph(vertex_slots[i], glyphs::CIRCLE, color::RED);
			}
		}

		for (auto& line : size_lines) {
			for (int s : line.first) {
				set_glyph(half_edge_slots[s], glyphs::SOLID_LINE, color::GREEN);
			}
		}

	}

	void CoarseKnitMesh::save(std::ofstream& f) {
		LabeledQuadMesh::save(f);

		// Save Seams
		f << seams.size() << std::endl;
		for (auto& s : seams) {
			s->save(f);
		}
		hlk::save(f, seam_edges);

		hlk::save(f, vertex_in_seam);

		hlk::save(f, size_lines);

		hlk::save(f, symmetries);

		hlk::save(f, side_lengths);
		

		// Poperties
		for (auto& e : edges) {
			e.save(f);
		}
		for (auto& q : quads) {
			q.save(f);
		}
		for (auto& s : sides) {
			s.save(f);
		}

		// Other State
		f << scale << " " << stitch_gauge << " " << row_gauge << " " <<
			tolerance << " " << min_time << " " << max_time << " " <<
			minimizer_timeout << " " << topology_solved << " " << geometry_solved;

		hlk::save(f, vertex_is_special);
	}

	void CoarseKnitMesh::load(std::ifstream& f) {
		LabeledQuadMesh::load(f);
		// Setup the optimizers, but don't run them since we'll just be loading state
		init(false);

		// Load the Seams, creating more if the user has edited them
		int num_seams;
		f >> num_seams;
		while (seams.size() < num_seams) {
			seams.emplace_back(topology_optimizer.get_bool_prop(nth_label("seam", seams.size())));
		}

		for (auto& s : seams) {
			s->load(f);
		}

		hlk::load(f, seam_edges);

		hlk::load(f, vertex_in_seam);

		hlk::load(f, size_lines);

		hlk::load(f, symmetries);

		hlk::load(f, side_lengths);

		// Properties
		for (auto& e : edges) {
			e.load(f);
		}
		for (auto& q : quads) {
			q.load(f);
		}
		for (auto& s : sides) {
			s.load(f);
		}

		// Other State
		f >> scale >> stitch_gauge >>  row_gauge >>
			tolerance  >> min_time >> max_time >>
			minimizer_timeout >> topology_solved >> geometry_solved;

		// Optional - older files won't have it
		if (f && f.peek() != EOF) {
			hlk::load(f, vertex_is_special);
		}
	}

	void CoarseKnitMesh::init()
	{
		init(false);
	}

	void CoarseKnitMesh::init(bool do_opt)
	{
		LabeledQuadMesh::init();

		// Clean up from any previous inits
		seams.clear();
		vertex_in_seam.clear();
		size_lines.clear();
		symmetries.clear();
		edges.clear();
		quads.clear();
		sides.clear();
		geometry_optimizer.clear();
		topology_optimizer.clear();
		side_lengths.clear();
		vertex_is_special.clear();

		// TODO - get side lengths from original mesh instead of coarse mesh
		for (int q = 0; q < m; ++q) {
			for (int i = 0; i < 4; ++i) {
				double len = (V.row(F_q(q, (i + 1) % 4)) - V.row(F_q(q, i))).norm();
				side_lengths.push_back(len);
			}
		}

		for (int i = 0; i < e; ++i) {
			edges.emplace_back(topology_optimizer, geometry_optimizer, i, this);
		}

		for (int q = 0; q < m; ++q) {
			
			for (int j = 0; j < 4; ++j) {
				sides.emplace_back(topology_optimizer, geometry_optimizer, 4 * q + j, this);
			}
			quads.emplace_back(topology_optimizer, geometry_optimizer, q, this);
			
		}
		

		vertex_in_seam.resize(n, false);
		vertex_is_special.resize(n, false);

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
							seams.emplace_back(topology_optimizer.get_bool_prop(nth_label("seam", sep_seam)));
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

		
		update_textures();

		// Solve the SMT problem and update the textures
		if (do_opt) {
			std::cout << "Mesh Loaded, Optimizing Topology" << std::endl;
			if (!optimize_topology()) {
				std::cout << "Unable to initialize" << std::endl;
			}
		}

	}


	CoarseKnitEdge::CoarseKnitEdge(Optimizer & topo_opt, Optimizer & geo_opt, int i, CoarseKnitMesh * m)
	{
		seam = -1;
		index = i;
		mesh = m;
	}

	std::vector<std::pair<z3::expr, std::string>> CoarseKnitEdge::get_topology_constraints()
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
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitEdge::get_geometry_constraints()
	{
		std::vector<std::pair<z3::expr, std::string>> constraints;

		return constraints;
	}

	int CoarseKnitEdge::target_stitch_count()
	{
		auto& side = mesh->sides[mesh->edges_to_sides(index, 0)];
		return side.target_stitch_count();
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
	void CoarseKnitEdge::save(std::ofstream& f)
	{
		f << seam << " " << index << std::endl;
	}
	void CoarseKnitEdge::load(std::ifstream& f)
	{
		f >> seam >> index;
	}
	CoarseKnitQuad::CoarseKnitQuad(Optimizer & topo_opt, Optimizer & geo_opt, int i, CoarseKnitMesh * m)
	{
		shaping_distribution = BOTH_SIDES;
		short_row_distribution = NONE;
		index = i;
		mesh = m;
		time = topo_opt.get_int_prop(nth_label("time", i));
	}
	std::vector<std::vector<int>> CoarseKnitQuad::get_symmetries()
	{
		std::vector<std::vector<int>> side_map(4);
		for (int i = 0; i < 4; ++i) {
			int s = 4 * index + i;
			auto& side = mesh->sides[s];
			side_map[side.generalized_index()].push_back(s);
		}

		bool inc_dec_allowed = shaping_distribution != NONE;
		bool sr_allowed = short_row_distribution != NONE;

		if (side_map[0].size() == 0 || side_map[2].size() == 0) {
			sr_allowed = false;
		}

		if (side_map[1].size() == 0 || side_map[3].size() == 0) {
			inc_dec_allowed = false;
		}

		std::vector<std::vector<int>> symmetries;

		if (!inc_dec_allowed && side_map[0].size() == 1 && side_map[2].size() == 1) {
			symmetries.push_back(std::vector<int>{side_map[0][0], side_map[2][0]});
		}
		if (!sr_allowed && side_map[1].size() == 1 && side_map[3].size() == 1) {
			symmetries.push_back(std::vector<int>{side_map[1][0], side_map[3][0]});
		}

		return symmetries;
	}
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitQuad::get_topology_constraints()
	{
		int q = 4*this->index;
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

		auto has_yarn = !s0_l || !s1_l || !s2_l || !s3_l;

		auto criss_cross = s0_l && s1_l && s2_l && s3_l && (s0_o != s2_o) && (s1_o != s3_o);

		std::vector<std::pair<z3::expr, std::string>> constraints;
		
		int num_nonreg = 0;
		for (int i = 0; i < 4; ++i) {
			int u = mesh->side_u(q + i);
			if (mesh->valence[u] % 4 != 0 || mesh->vertex_is_special[u]) {
				++num_nonreg;
			}
		}
		
		if (num_nonreg >= 2) {
			/*
			constraints.push_back(std::make_pair(
				no_skip_0,
				"no_skip_0_" + std::to_string(q)
			));

			constraints.push_back(std::make_pair(
				no_skip_1,
				"no_skip_1_" + std::to_string(q)
			));
			*/
		}
		
		if (num_nonreg >= 4) {
			constraints.push_back(std::make_pair(
				!criss_cross,
				"no_criss_cross_" + std::to_string(q)
			));
			constraints.push_back(std::make_pair(
				has_yarn || sink_source,
				"has_yarn_or_sink_source_" + std::to_string(q)
			));
		}


		
		/*
		constraints.push_back(std::make_pair(
			!sink_source,
			"no_sink_source_" + std::to_string(q)
		));
		*/

		return constraints;
	}
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitQuad::get_geometry_constraints()
	{

		std::vector<std::pair<z3::expr, std::string>> constraints;

		std::vector<std::vector<z3::expr>> side_exprs(4);
		for (int i = 0; i < 4; ++i) {
			int s = 4 * index + i;
			auto& side = mesh->sides[s];
			side_exprs[side.generalized_index()].push_back(side.stitches->var);
		}

		std::vector<z3::expr> side_stitches;
		for (int i = 0; i < 4; ++i) {
			if (side_exprs[i].size() == 0) {
				side_stitches.push_back(mesh->geometry_optimizer.one());
			}
			else {
				side_stitches.push_back(side_exprs[i][0]);
			}
			for (int j = 1; j < side_exprs[i].size(); ++j) {
				side_stitches.back() = side_stitches.back() + side_exprs[i][j];
			}

			//side_stitches.push_back(mesh->geometry_optimizer.zero());
		}
		//std::vector<std::vector<z3::expr>> side_vars(4);
		/*
		for (int i = 0; i < 4; ++i) {
			auto& side = mesh->sides[4 * index + i];
			int side_idx = side.generalized_index();
			side_stitches[side_idx] = side_stitches[side_idx] + side.stitches->var;
			//side_vars[side_idx].push_back(side.stitches->var);
		}
		*/

		auto& loop_in = side_stitches[0];
		auto& yarn_out = side_stitches[1];
		auto& loop_out = side_stitches[2];
		auto& yarn_in = side_stitches[3];

		auto one_shaping_type = loop_in == loop_out || yarn_in == yarn_out;
		auto cols = z3::ite(loop_in > loop_out, loop_in, loop_out);
		auto rows = z3::ite(yarn_in > yarn_out, yarn_in, yarn_out);
		auto loop_min = z3::ite(loop_in > loop_out, loop_out, loop_in);
		auto loop_max = z3::ite(loop_in > loop_out, loop_in, loop_out);
		auto yarn_min = z3::ite(yarn_in > yarn_out, yarn_out, yarn_in);
		auto yarn_max = z3::ite(yarn_in > yarn_out, yarn_in, yarn_out);


		// Here are the asserts that we want

		/*
		if (shaping == NONE) {
			assert(loop_in == loop_out);
		}
		else {
			assert(loop_in + loop_in * (yarn_in - 1) >= loop_out);
			assert(loop_in + loop_in * (yarn_out - 1) >= loop_out);
			assert(loop_out + loop_out * (yarn_in - 1) >= loop_in);
			assert(loop_out + loop_out * (yarn_out - 1) >= loop_in);
		}
		if (sr_shaping == LOOP_IN) {
			assert(yarn_in + loop_in >= yarn_out);
			assert(yarn_out + loop_in >= yarn_in);
		}
		else if (sr_shaping == LOOP_OUT) {
			assert(yarn_in + loop_out >= yarn_out);
			assert(yarn_out + loop_out >= yarn_in);
		}
		else { // sr_shaping == NONE
			assert(yarn_in == yarn_out);
		}
		*/
		

		// Only put constraints on if we definitely need them
		
		bool inc_dec_allowed = shaping_distribution != NONE;
		bool sr_allowed = short_row_distribution != NONE;

		/*
		if (side_exprs[0].size() == 0 || side_exprs[2].size() == 0) {
			sr_allowed = false;
		}

		if (side_exprs[1].size() == 0 || side_exprs[3].size() == 0) {
			inc_dec_allowed = false;
		}
		*/

		if (!sr_allowed) {
			// If this is a normal quad with none allowed, this constraint is handled
			// already by merging variables
			if (side_exprs[1].size() != 0 || side_exprs[3].size() != 0) {
				constraints.push_back(std::make_pair(
					yarn_in == yarn_out,
					"no_short_row_" + std::to_string(index)
				));
			}
		}
		if (!inc_dec_allowed) {
			// If this is a normal quad with none allowed, this constraint is handled
			// already by merging variables
			if (side_exprs[0].size() != 0 || side_exprs[2].size() != 0) {
				constraints.push_back(std::make_pair(
					loop_in == loop_out,
					"no_inc_dec_" + std::to_string(index)
				));
			}
		}
		
		/*
		if (inc_dec_allowed && sr_allowed) {
			constraints.push_back(std::make_pair(
				one_shaping_type,
				"one_shaping_type_" + std::to_string(index)
			));

		}
		*/
		
		if (inc_dec_allowed && !sr_allowed) {
			constraints.push_back(std::make_pair(
				(loop_in + loop_in * (yarn_in - 1) >= loop_out) &&
				(loop_out + loop_out * (yarn_in - 1) >= loop_in),
				"gentle_inc_dec_only_" + std::to_string(index)
			));
		}

		if (sr_allowed && !inc_dec_allowed) {
			constraints.push_back(std::make_pair(
				(yarn_in + loop_in >= yarn_out) &&
				(yarn_out + loop_in >= yarn_in),
				"gentle_short_row_only_" + std::to_string(index)
			));
		}

		if (inc_dec_allowed && sr_allowed) {
			constraints.push_back(std::make_pair(
				(loop_in + loop_in * (yarn_in - 1) >= loop_out) &&
				(loop_out + loop_out * (yarn_in - 1) >= loop_in) &&
				(loop_in + loop_in * (yarn_out - 1) >= loop_out) &&
				(loop_out + loop_out * (yarn_out - 1) >= loop_in),
				"gentle_inc_dec_" + std::to_string(index)
			));
			if (short_row_distribution == IN_SIDE) {
				constraints.push_back(std::make_pair(
					(yarn_in + loop_in >= yarn_out) &&
					(yarn_out + loop_in >= yarn_in),
					"gentle_short_row_in_" + std::to_string(index)
				));
			}
			if (short_row_distribution == OUT_SIDE) {
				constraints.push_back(std::make_pair(
					(yarn_in + loop_out >= yarn_out) &&
					(yarn_out + loop_out >= yarn_in),
					"gentle_short_row_out_" + std::to_string(index)
				));
			}
		}
		
		return constraints;
	}
	void CoarseKnitQuad::get_corners(Eigen::MatrixXd& C) const
	{
		C.resize(4, 3);
		for (int i = 0; i < 4; ++i) {
			C.row(i) = mesh->V.row(mesh->F_q(index, i));
		}
	}
	void CoarseKnitQuad::get_generalized_corners(Eigen::MatrixXd& C) const
	{
		Eigen::MatrixXd temp;
		get_corners(temp);
		C.resizeLike(temp);
		for (int i = 0; i < C.rows(); ++i) {
			C.row(to_generalized(i)) = temp.row(i);
		}
	}
	int CoarseKnitQuad::get_generalized_bottom_left() const
	{
		std::vector<bool> is_out;
		std::vector<bool> is_loop;
		for (int i = 0; i < 4; ++i) {
			is_loop.push_back(mesh->sides[4*index + i].is_loop->val);
			is_out.push_back(mesh->sides[4*index + i].is_out->val);
		}
		std::vector<int> split_points;
		std::vector<int> split_sides;
		// Helper to get a generalized side index 0 = loop_in, 1 = yarn_out, 2 = loop_out, 3 = yarn_in
		auto idx = [](bool is_loop, bool is_out)->int {
			return ((is_loop ? 0 : 3) + (is_out ? 2 : 0)) % 4;
		};

		for (int n = 0; n < 4; ++n) {
			int p = (n + 3) % 4; // Previous side
			if (is_loop[p] != is_loop[n] || is_out[p] != is_out[n]) {
				// Side type changes around vertex n
				split_points.push_back(n);
				split_sides.push_back(idx(is_loop[n], is_out[n]));
			}
		}
		// split_points now contains vtx indices where generalized quad side type changes
		// split_sides contains which generalized side comes next
		// We want to find the corner before the first generalized side
		// If there are no splits, we are at a source or sink and arbitrarily choose 0
		int min_idx = 0;
		for (int i = 0; i < split_sides.size(); ++i) {
			if (split_sides[i] < split_sides[min_idx]) {
				min_idx = i;
			}
		}
		return split_points[min_idx];
	}
	void CoarseKnitQuad::get_sides_stitches(std::vector<std::vector<int>>& sides) const
	{
		int c = get_generalized_bottom_left();
		sides.resize(4);
		for (int i = 0; i < 4; ++i) {
			auto& side = mesh->sides[4*index + ((c + i) % 4)];
			sides[side.generalized_index()].push_back(side.stitches->val);
		}
	}
	int CoarseKnitQuad::to_generalized(int side) const
	{
		int c = get_generalized_bottom_left();
		return (side - c + 4) % 4;
	}
	void CoarseKnitQuad::update_texture()
	{
		auto slot = mesh->quad_slots[index];
		auto tex = get_textures();
		mesh->set_glyph(slot, glyphs::SOLID_LINE, tex[texture_id].color);
		// TODO - Print Glyphs for inc/dec type

		// First identify orientation
		std::vector<int> gen_indices;
		std::vector<int> side_locs(4, -1);
		int top = -1, bottom = -1, left = -1, right = -1;
		for (int i = 0; i < 4; ++i) {
			gen_indices.push_back(mesh->sides[index * 4 + i].generalized_index());
			side_locs[gen_indices.back()] = i;
		}

		switch (short_row_distribution) {
		case ShapingType::NONE:
			if (side_locs[0] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[0]), glyphs::NO_SHORT_ROW, color::GREY);
			}
			else if (side_locs[2] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[2]), glyphs::NO_SHORT_ROW, color::GREY);
			}
			break;
		case ShapingType::IN_SIDE:
			if (side_locs[0] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[0]), glyphs::SHORT_ROW, color::GREY);
			}
			break;
		case ShapingType::OUT_SIDE:
			if (side_locs[2] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[2]), glyphs::SHORT_ROW, color::GREY);
			}
			break;
		}

		switch (shaping_distribution) {
		case ShapingType::NONE:
			if (side_locs[1] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[1]), glyphs::NO_INCREASE, color::GREY);
			}
			else if (side_locs[3] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[3]), glyphs::NO_INCREASE, color::GREY);
			}
			break;
		case ShapingType::IN_SIDE:
			if (side_locs[3] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[3]), glyphs::LEANING_INCREASE, color::GREY);
			}
			break;
		case ShapingType::OUT_SIDE:
			if (side_locs[1] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[1]), glyphs::LEANING_INCREASE, color::GREY);
			}
			break;
		case ShapingType::BOTH_SIDES:
			if (side_locs[1] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[1]), glyphs::LEANING_INCREASE, color::GREY);
			}
			if (side_locs[3] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[3]), glyphs::LEANING_INCREASE, color::GREY);
			}
			break;
		case ShapingType::DISTRIBUTED:
			if (side_locs[1] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[1]), glyphs::INCREASE, color::GREY);
			}
			else if (side_locs[3] >= 0) {
				mesh->set_glyph(mesh->quadrant_slot(index, side_locs[3]), glyphs::INCREASE, color::GREY);
			}
			break;
		}

	}
	void CoarseKnitQuad::save(std::ofstream& f)
	{
		time->save(f);
		f << index << " " << (int)shaping_distribution << " " << (int)short_row_distribution << " " << texture_id << std::endl;
	}
	void CoarseKnitQuad::load(std::ifstream& f)
	{
		time->load(f);
		int sd, srd;
		f >> index >> sd >> srd >> texture_id;
		shaping_distribution = (ShapingType)sd;
		short_row_distribution = (ShapingType)srd;

	}
	CoarseKnitSide::CoarseKnitSide(Optimizer & topo_opt, Optimizer & geo_opt, int i, CoarseKnitMesh * m)
	{
		is_loop = topo_opt.get_bool_prop(nth_label("is_loop", i));
		is_out = topo_opt.get_bool_prop(nth_label("is_out", i));
		index = i;
		mesh = m;
		
		stitches = geo_opt.get_int_prop(nth_label("side_stitches", i));
	}

	double CoarseKnitSide::get_gauge()
	{
		return is_loop->val ? mesh->stitch_gauge : mesh->row_gauge;;
	}

	void CoarseKnitSide::cache_stitches()
	{
		stitches_backup = stitches;
	}

	void CoarseKnitSide::uncache_stitches()
	{
		stitches_backup->val = stitches->val;
		is_representative = true;
		stitches = stitches_backup;
	}

	int CoarseKnitSide::target_stitch_count()
	{
		double target_length = mesh->side_lengths[index];
		double gauge = get_gauge();
		int target_stitches = round(target_length * gauge / mesh->scale);
		target_stitches = target_stitches > 0 ? target_stitches : 1; // At least 1 stitch per side
		return target_stitches;
	}

	std::vector<std::pair<z3::expr,std::string>> CoarseKnitSide::get_topology_constraints()
	{

		std::vector<std::pair<z3::expr, std::string>> constraints;
		int u = mesh->side_u(index);
		bool is_border = mesh->is_border_vertex[u];
		int valence = mesh->valence[u];

		// For our purposes, valences of multiples of 4 (3 on borders)
		// do not count as singularities, so calculate this explicitly
		bool is_regular = 
			((is_border && (valence % 3 == 0)) || 
			(valence % 4 == 0)) && 
			!mesh->vertex_is_special[u];
		int prev_idx = mesh->prev_side(index);
		auto& prev_is_loop = mesh->sides[prev_idx].is_loop;
		auto& prev_is_out = mesh->sides[prev_idx].is_out;
		if (is_regular) {
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

		}
		else {
			// Constrain that the order at a singular corner can't go "backwards"
			// by 1. This looks like the the above constraint, except that loop/yarn are
			// switched (going "backward") and the entire term is negated (prohibit that path)

			auto no_double_yarn = (is_out == prev_is_out) == is_loop->var;
			auto skip_yarn = (is_out != prev_is_out) && is_loop;

			constraints.push_back(
				std::make_pair(
					no_double_yarn || skip_yarn,
					"singular_corner_" + std::to_string(index)
				)
			);

		}
		return constraints;
	}
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitSide::get_geometry_constraints()
	{
		std::vector<std::pair<z3::expr, std::string>> constraints;

		// Only add for boundary edges
		if (is_representative) {
			int target_stitches = target_stitch_count();
			int tolerance = ceil(mesh->tolerance * target_stitches);
			if (mesh->edge_tolerance > 0) {
				tolerance = mesh->edge_tolerance;
			}
			if (mesh->course_tolerance > 0 && mesh->wale_tolerance > 0) {
				if (get_gauge() == mesh->row_gauge) {
					tolerance = mesh->wale_tolerance;
				}
				else {
					tolerance = mesh->course_tolerance;
				}
			}
			int min_sts = target_stitches - tolerance;
			int max_sts = target_stitches + tolerance;
			min_sts = min_sts > 0 ? min_sts : 1;

		
			constraints.push_back(std::make_pair(
				min_sts <= stitches->var,
				"min_sts_side_" + std::to_string(index)
			));

			constraints.push_back(std::make_pair(
				max_sts >= stitches->var,
				"max_sts_side_" + std::to_string(index)
			));
			
		}

		return constraints;
	}
	z3::expr CoarseKnitSide::get_geometry_cost()
	{
		int target_stitches = target_stitch_count();
		return (stitches->var - target_stitches) * (stitches->var - target_stitches);
	}
	void CoarseKnitSide::update_texture()
	{
		auto& arrow = is_loop->is_fixed ? glyphs::SOLID_ARROW : glyphs::DASHED_ARROW;
		auto& line = is_loop->is_fixed ? glyphs::THIN_SOLID_LINE : glyphs::THIN_DASHED_LINE;
		auto& s_color = is_loop->val ? color::ORANGE : color::GREEN;
		auto& symbol = is_out->val ? line : arrow;
		mesh->set_glyph(mesh->dual_half_edge_slots[index], symbol, s_color);
	}
	int CoarseKnitSide::generalized_index()
	{
		return ((is_loop->val ? 0 : 3) + (is_out->val ? 2 : 0)) % 4;
	}
	void CoarseKnitSide::save(std::ofstream& f)
	{
		is_loop->save(f);
		is_out->save(f);
		stitches->save(f);
		f << index << std::endl;
	}
	void CoarseKnitSide::load(std::ifstream& f)
	{
		is_loop->load(f);
		is_out->load(f);
		stitches->load(f);
		f >> index;
	}
	bool CoarseKnitMesh::optimize_topology()
	{
		if (topology_solved) return true;
		topology_optimizer.push();

		std::vector<z3::expr> seam_costs;
		int i = 0;
		std::cout << "Num possible seams = " << seams.size() << std::endl;
		for (auto& seam : seams) {
			std::string cost_name = "seam_cost_" + std::to_string(i);
			z3::expr s_cost = topology_optimizer.context.int_const(cost_name.c_str());
			seam_costs.push_back(s_cost);
			int seam_length = seam_edges[i].size();
			topology_optimizer.add_constraint((seam->var && s_cost == seam_length) || (!seam->var && s_cost == 0));
			++i;
		}


		for (auto& edge : edges) {
			for (auto constraint : edge.get_topology_constraints()) {
				topology_optimizer.add_constraint(constraint.first, constraint.second);
			}
		}
		for (auto& side : sides) {
			for (auto constraint : side.get_topology_constraints()) {
				topology_optimizer.add_constraint(constraint.first, constraint.second);
			}
		}

		for (auto& quad : quads) {
			for (auto constraint : quad.get_topology_constraints()) {
				topology_optimizer.add_constraint(constraint.first, constraint.second);
			}
		}



		z3::expr cost = topology_optimizer.context.int_const("cst");
		if (seam_costs.size() > 0) {
			cost = seam_costs[0];
			for (int i = 1; i < seam_costs.size(); ++i) {
				cost = cost + seam_costs[i];
			}
		}

		auto result = seam_costs.size() > 0 ? topology_optimizer.minimize(cost, minimizer_timeout) : topology_optimizer.solve();

		if (result.has_result) {
			topology_optimizer.update_all_props(*result.result_model);
			update_textures();
			topology_solved = true;
		}
		else {
			// TODO - Get Information from the UNSAT core
			std::cout << result.unsat_core << std::endl;
			topology_solved = false;
		}

		topology_optimizer.pop();
		return result.has_result;
	}

	bool CoarseKnitMesh::optimize_geometry()
	{
		//if (geometry_solved) return true;
		if (!topology_solved) return false;

		// Symmetry optimization - unify symmetric variables!
		for (auto& side : sides) {
			side.cache_stitches();
		}

		std::vector<std::vector<int>> side_symmetries;

		for (auto& symmetry : symmetries) {
			side_symmetries.push_back(symmetry);
		}

		for (auto& q : quads) {
			for (auto sym : q.get_symmetries()) {
				side_symmetries.push_back(sym);
			}
		}

		for (auto& edge : edges) {
			if (edge.seam < 0 || !seams[edge.seam]->val) {
				side_symmetries.push_back({ edges_to_sides(edge.index, 0), edges_to_sides(edge.index, 1) });
			}
		}

		if (symmetrize) {
			Symmetrizer symmetrizer(V, F_t);
			for (int axis = 0; axis < 3; ++axis) {
				if (symmetrizer.has_vertex_symmetry(axis)) {
					for (int i = 0; i < sides.size(); ++i) {
						int u = side_u(i);
						int v = side_v(i);
						int u_sym = symmetrizer.symmetric_vertex(u, axis);
						int v_sym = symmetrizer.symmetric_vertex(v, axis);
						for (int s : out_sides(u_sym)) {
							if (side_v(s) == v_sym) {
								side_symmetries.push_back({ i, s });
							}
						}
						for (int s : in_sides(u_sym)) {
							if (side_u(s) == v_sym) {
								side_symmetries.push_back({ i,s });
							}
						}
					}
				}
			}
		}
		

		// Now minimize the symmetries by merging

		IntUnionFind djs(sides.size());
		for (auto& sym : side_symmetries) {
			for (int i = 1; i < sym.size(); ++i) {
				djs.join(sym[0], sym[i]);
			}
		}
		std::vector<std::vector<int>> components = djs.components();

		for (auto& symmetry : components) {
			auto representative = sides[symmetry[0]].stitches;
			for (int i = 1; i < symmetry.size(); ++i) {
				sides[symmetry[i]].stitches = representative;
				sides[symmetry[i]].is_representative = false;
			}
		}

		
		bool done = false;

		int starting_tolerance = edge_tolerance;
		while (!done) {

			geometry_optimizer.push();

			z3::expr cost = sides[0].get_geometry_cost();
			
			for (int i = 1; i < sides.size(); ++i) {
				int opp = flip_side(i);
				if (i < opp || opp < 0) {
					cost = cost + sides[i].get_geometry_cost();
				}
			}
			std::cout << "Cost Function = " << std::endl << cost.to_string() << std::endl;

			for (auto& side : sides) {
				for (auto constraint : side.get_geometry_constraints()) {
					geometry_optimizer.add_constraint(constraint.first, constraint.second);
					std::cout << "Added Constraint: " << constraint.second << std::endl << constraint.first.to_string() << std::endl;
				}
			}

			for (auto& quad : quads) {
				for (auto constraint : quad.get_geometry_constraints()) {
					geometry_optimizer.add_constraint(constraint.first, constraint.second);
					std::cout << "Added Constraint: " << constraint.second << std::endl << constraint.first.to_string() << std::endl;
				}
			}

			for (auto& edge : edges) {
				for (auto constraint : edge.get_geometry_constraints()) {
					geometry_optimizer.add_constraint(constraint.first, constraint.second);
					std::cout << "Added Constraint: " << constraint.second << std::endl << constraint.first.to_string() << std::endl;
				}
			}

			for (auto constraint : size_line_constraints()) {
				geometry_optimizer.add_constraint(constraint.first, constraint.second);
				std::cout << "Added Constraint: " << constraint.second << std::endl << constraint.first.to_string() << std::endl;
			}

			auto result = geometry_optimizer.minimize(cost, minimizer_timeout);

			if (result.has_result) {
				geometry_optimizer.update_all_props(*result.result_model);
				update_textures();
				geometry_solved = true;
				done = true;
			}
			else {
				// TODO - Get Information from the UNSAT core
				std::cout << result.unsat_core << std::endl;
				geometry_solved = false;
				std::cout << "Increasing Tolerance to " << edge_tolerance + 1 << std::endl;
				++edge_tolerance;
				if (edge_tolerance > starting_tolerance + 1) {
					done = true;
					std::cout << "Stopping increasing tolerance." << std::endl;
				}
			}

			geometry_optimizer.pop();
		}

		// Return variables to normal
		for (auto& side : sides) {
			side.uncache_stitches();
		}

		return geometry_solved;
	}
	CoarseKnitGraph CoarseKnitMesh::get_dual()
	{
		CoarseKnitGraph graph;
		

		for (int q = 0; q < quads.size(); ++q) {
			Eigen::MatrixXd corners;
			std::vector<std::vector<int>> stitch_counts;
			quads[q].get_generalized_corners(corners);
			quads[q].get_sides_stitches(stitch_counts);
			graph.patch_data.emplace_back(stitch_counts, corners, quads[q].time->val);
			graph.patch_data.back().texture_id = quads[q].texture_id;
			graph.patch_data.back().shaping = (int)quads[q].shaping_distribution;
			graph.patch_data.back().short_row_shaping = (int)quads[q].short_row_distribution;
		}

		for (int e = 0; e < edges.size(); ++e) {
			int seam = edges[e].seam;
			if (seam < 0 || !seams[seam]->val) {
				Eigen::RowVector2i e_sides = edges_to_sides.row(e);
				int src = e_sides[0];
				int dst = e_sides[1];
				if (!sides[e_sides[0]].is_out->val) {
					src = e_sides[1];
					dst = e_sides[0];
				}
				CoarseKnitGraph::Edge edge;
				edge.src = src / 4;
				edge.dst = dst / 4;
				
				int raw_src_side = src % 4;
				int raw_dst_side = dst % 4;

				int src_c = quads[edge.src].get_generalized_bottom_left();
				int dst_c = quads[edge.dst].get_generalized_bottom_left();

				edge.src_side = (raw_src_side - src_c + 4) % 4;
				edge.dst_side = (raw_dst_side - dst_c + 4) % 4;
				edge.is_loop = sides[src].is_loop->val;
				graph.edges.push_back(edge);
			}
		}

		return graph;
	}
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitMesh::get_seam_costs()
	{
		return std::vector<std::pair<z3::expr, std::string>>();
	}
	double CoarseKnitMesh::add_size_line(std::vector<int> line_sides, double target)
	{
		double size = 0;
		if (target < 0) {
			for (int i = 0; i < line_sides.size(); ++i) {
				size += side_lengths[line_sides[i]];
			}
		}
		else {
			size = target;
		}
		auto size_line = std::make_pair(line_sides, size);
		size_lines.push_back(size_line);
		std::cout << "Added a size line with target size of " << size << std::endl;
		update_textures();
		return size;
	}

	std::vector<std::pair<z3::expr, std::string>> CoarseKnitMesh::size_line_constraints()
	{
		std::vector<std::pair<z3::expr, std::string>> constraints;

		for (int l = 0; l < size_lines.size(); ++l) {
			auto& size_line = size_lines[l];
			auto& line = size_line.first;
			auto target = size_line.second / scale;

			auto length = sides[line[0]].stitches->var;
			
			for (int i = 1; i < line.size(); ++i) {
				length = length + sides[line[i]].stitches->var;
			}

			double gauge = sides[line[0]].is_loop->val ? stitch_gauge : row_gauge;

			int target_stitches = round(target * gauge);
			int toll = critical_tolerance > 0 ? critical_tolerance : round(tolerance * target_stitches);
			int min_stitches = target_stitches - toll;
			int max_stitches = target_stitches + toll;
			min_stitches = min_stitches > 0 ? min_stitches : 1;
			max_stitches = max_stitches > 0 ? max_stitches : 1;

			constraints.push_back(std::make_pair(
				length >= min_stitches,
				"size_line_min_" + std::to_string(l)
			));

			constraints.push_back(std::make_pair(
				length <= max_stitches,
				"size_line_max_" + std::to_string(l)
			));
		}

		return constraints;
	}
	std::vector<std::pair<z3::expr, std::string>> CoarseKnitMesh::get_symmetry_constraints()
	{
		std::vector<std::pair<z3::expr, std::string>> constraints;

		for (int i = 0; i < symmetries.size(); ++i) {
			auto& symmetry = symmetries[i];
			auto& rep_stitches = sides[symmetry[0]].stitches;
			for (int j = 1; j < symmetry.size(); ++j) {
				auto& stitches = sides[symmetry[i]].stitches;
				constraints.push_back(std::make_pair(
					rep_stitches == stitches,
					"symmetry_" + std::to_string(i) + "_" + std::to_string(j)
				));
			}
		}

		return constraints;
	}
	void CoarseKnitMesh::copy_shaping(int origin_side, int dest_side)
	{
		auto& origin_quad = quads[origin_side / 4];
		auto& dest_quad = quads[dest_side / 4];

		dest_quad.shaping_distribution = origin_quad.shaping_distribution;
		dest_quad.short_row_distribution = origin_quad.short_row_distribution;

		geometry_solved = false;
		update_textures();
	}
	void CoarseKnitMesh::set_shaping(int side, ShapingType shaping, ShapingType short_row)
	{
		auto& dest_quad = quads[side / 4];
		dest_quad.shaping_distribution = shaping;
		dest_quad.short_row_distribution = short_row;

		geometry_solved = false;
		update_textures();
	}
	void CoarseKnitMesh::set_texture(int side, int texture)
	{
		quads[side / 4].texture_id = texture;
		update_textures();
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

		topology_solved = false;
		geometry_solved = false;
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

		topology_solved = false;
		geometry_solved = false;
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

		topology_solved = false;
		geometry_solved = false;
	}
	void CoarseKnitMesh::toggle_special_vertex(int vertex)
	{
		// Don't allow toggling of singularities
		std::cout << "Toggling Vertex " << vertex << std::endl;
		if (!is_singularity[vertex]) {
			vertex_is_special[vertex] = !vertex_is_special[vertex];
			update_textures();
		}
	}
	void CoarseKnitMesh::add_seam(std::vector<int> seam_sides)
	{
		for (auto side : seam_sides) {
			assert(sides_to_edges[side] >= 0);
			assert(edges[sides_to_edges[side]].seam < 0);
		}
		int seam_id = seams.size();
		seams.emplace_back(topology_optimizer.get_bool_prop(nth_label("seam", seam_id)));
		std::vector<int> new_seam;
		for (auto side : seam_sides) {
			vertex_in_seam[side_u(side)] = true;
			vertex_in_seam[side_v(side)] = true;
			int e = sides_to_edges[side];
			edges[e].seam = seam_id;
			new_seam.push_back(e);
		}
		seam_edges.push_back(new_seam);

		topology_solved = false;
		geometry_solved = false;
	}

	std::vector<int> CoarseKnitMesh::trace_seam(int side) {

		auto loop_sides = side_loop(side);
		int last = 0;
		while (last < loop_sides.size() && !vertex_in_seam[side_v(loop_sides[last])]) ++last;
		std::vector<int> seam_edges;
		for (int i = 0; i <= last && i < loop_sides.size(); ++i) {
			seam_edges.push_back(sides_to_edges[loop_sides[i]]);
		}
		return seam_edges;
	}

	void CoarseKnitMesh::debug_label_vertices(igl::opengl::ViewerData* debug)
	{
		for (int i = 0; i < n; ++i) {
			debug->add_label(V.row(i), std::to_string(i));
		}
	}

	void CoarseKnitMesh::debug_label_edges(igl::opengl::ViewerData* debug)
	{
		for (int i = 0; i < e; ++i) {
			debug_label_edge(debug, i);
		}
	}

	void CoarseKnitMesh::debug_label_edge(igl::opengl::ViewerData* debug, int edge)
	{
		if (edge < 0 || edge >= e) {
			std::cout << "Trying to label " << e << " which is not an edge." << std::endl;
		}
		else {
			int s = edges_to_sides(edge, 0);
			Eigen::Vector3d pos = (V.row(side_u(s)) + V.row(side_v(s))) / 2;
			debug->add_label(pos, std::to_string(e));
		}
	}

	void CoarseKnitMesh::debug_label_sides(igl::opengl::ViewerData* debug)
	{
		Eigen::MatrixXd BC;
		igl::barycenter(V, F_t, BC);
		for (int i = 0; i < BC.rows(); ++i) {
			debug->add_label(BC.row(i), std::to_string(i));
		}
	}

	void CoarseKnitMesh::split_seams(int vertex_a, int vertex_b)
	{
		// Since split_seams doesn't do anything if a seam doesn't
		// contain a vertex, try to split all the seams
		std::vector<int> split_vtcs{ vertex_a, vertex_b };
		for (int i = 0; i < seams.size(); ++i) {
			split_seam(i, split_vtcs);
		}

		topology_solved = false;
		geometry_solved = false;
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
seams.emplace_back(topology_optimizer.get_bool_prop(nth_label("seam", new_seam_id)));
seam_edges.push_back(new_seams[i]);
for (int e : new_seams[i]) {
	edges[e].seam = new_seam_id;
}
		}


		topology_solved = false;
		geometry_solved = false;
	}
	void CoarseKnitMesh::toggle_orientation(int side)
	{
		sides[side].is_out->set(!sides[side].is_out->val);
		sides[side].is_loop->is_fixed = true;


		topology_solved = false;
		geometry_solved = false;
	}
	void CoarseKnitMesh::toggle_direction(int side)
	{
		sides[side].is_loop->set(!sides[side].is_loop->val);
		sides[side].is_out->is_fixed = true;

		topology_solved = false;
		geometry_solved = false;
	}
	void CoarseKnitMesh::paint_direction(int side_out, int side_in, KnitDirection dir)
	{
		bool is_loop = dir == LOOP;
		if (side_out >= 0) {
			sides[side_out].is_out->set(true);
			sides[side_out].is_loop->set(is_loop);
			sides[side_out].update_texture();
		}

		if (side_in >= 0) {
			sides[side_in].is_out->set(false);
			sides[side_in].is_loop->set(is_loop);
			sides[side_in].update_texture();
		}

		topology_solved = false;
		geometry_solved = false;
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

		topology_solved = false;
		geometry_solved = false;
	}
	void CoarseKnitMesh::erase_orientation(int side)
	{
		sides[side].is_loop->is_fixed = false;
		sides[side].is_out->is_fixed = false;
		sides[side].update_texture();

		topology_solved = false;
		geometry_solved = false;
	}
	void CoarseKnitMesh::erase_textue(int side)
	{
		quads[side / 4].texture_id = -1;
	}
	void CoarseKnitMesh::erase_shaping(int side)
	{
		quads[side / 4].shaping_distribution = NONE;
		quads[side / 4].short_row_distribution = NONE;

		geometry_solved = false;
	}

	void import_data(
		std::string filename,
		Eigen::MatrixXd& V,
		Eigen::MatrixXi& F,
		Eigen::VectorXi& F_sh, // Face Shaping 
		Eigen::VectorXi& F_sr, // Face Short Rows
		Eigen::VectorXi& F_tx, // Face Texture ID

		Eigen::MatrixXi& O_yl, // Orientation is yarn (0) or loop (1)
		Eigen::MatrixXi& O_io, // Orientation is in (0) or out (1)
		Eigen::MatrixXi& O_fx, // Orientation is fixed?

		Eigen::MatrixXi& S_op, // Side is part of a seam (not necessarily used)
		Eigen::MatrixXi& S_on, // Side's seam is used
		Eigen::MatrixXi& S_fx // Side's seam is fixed
		)
	{
		
		auto read_matrix_d = [](std::ifstream& f, Eigen::MatrixXd& M) {
			int r, c;
			f >> r >> c;
			M.resize(r, c);
			for (int i = 0; i < r; ++r) {
				for (int j = 0; j < c; ++j) {
					double v;
					f >> v;
					M(i, j) = v;
				}
			}
		};

		auto read_matrix_i = [](std::ifstream& f, Eigen::MatrixXi& M) {
			int r, c;
			f >> r >> c;
			M.resize(r, c);
			for (int i = 0; i < r; ++r) {
				for (int j = 0; j < c; ++j) {
					int v;
					f >> v;
					M(i, j) = v;
				}
			}
		};

		auto read_vector_i = [](std::ifstream& f, Eigen::VectorXi& V) {
			int r;
			f >> r;
			V.conservativeResize(r);
			for (int i = 0; i < r; ++r) {
				int v;
				f >> v;
				V(i) = v;
			}
		};

		
		std::ifstream file(filename);
		read_matrix_d(file, V);
		read_matrix_i(file, F);
		read_vector_i(file, F_sh);
		read_vector_i(file, F_sr);
		read_vector_i(file, F_tx);

		read_matrix_i(file, O_yl);
		read_matrix_i(file, O_io);
		read_matrix_i(file, O_fx);

		read_matrix_i(file, S_op);
		read_matrix_i(file, S_on);
		read_matrix_i(file, S_fx);

	}

	void CoarseKnitMesh::export_data(std::string filename)
	{
		// =============
		// Geometry Data
		// =============

		Eigen::MatrixXd V = this->V.block(0,0, n, 3); // Vertex Positions
		Eigen::MatrixXi F = this->F_q; // Face Vertices

		
		// =============
		// Per-Face Data
		// =============
		
		// Shaping and Short Rows:
		//   0 = NONE,
		//   1 = IN_SIDE,
		//   2 = OUT_SIDE,
		//   3 = BOTH_SIDES,
		//   4 = DISTRIBUTED
		// F_sr can only be 0-2
		Eigen::VectorXi F_sh(m); // Face Shaping 
		Eigen::VectorXi F_sr(m); // Face Short Rows
		Eigen::VectorXi F_tx(m); // Face Texture ID

		// =========================
		// Per Side (Half-Edge) Data
		// =========================

		// Orientation Data
		Eigen::MatrixXi O_yl(m, 4); // Orientation is yarn (0) or loop (1)
		Eigen::MatrixXi O_io(m, 4); // Orientation is in (0) or out (1)
		Eigen::MatrixXi O_fx(m, 4); // Orientation is fixed?

		// Seam Data
		// In our system, a "seam" is a path of edges that we can turn on or off as a unit.
		// Each of these seams can be "on" or "off", and also be fixed or not.
		// In this format, we have per-side information, so each side can be
		//   part of a seam?
		//   is that seam on or off?
		//   is that seam fixed?
		Eigen::MatrixXi S_op(m, 4); // Side is part of a seam (not necessarily used)
		Eigen::MatrixXi S_on(m, 4); // Side's seam is used
		Eigen::MatrixXi S_fx(m, 4); // Side's seam is fixed


		for (int i = 0; i < quads.size(); ++i) {
			auto& q = quads[i];
			F_sh(i) = (int) q.shaping_distribution;
			F_sr(i) = (int)q.short_row_distribution;
			F_tx(i) = q.texture_id;

			for (int j = 0; j < 4; ++j) {
				int k = 4 * i + j;
				
				O_yl(i, j) = (int)sides[k].is_loop->val;
				O_io(i, j) = (int)sides[k].is_out->val;
				O_fx(i, j) = (int)sides[k].is_loop->is_fixed;

				S_op(i, j) = 0;
				S_on(i, j) = 0;
				S_fx(i, j) = 0;
				int edge = sides_to_edges[k];
				if (edge >= 0) {
					int s = edges[edge].seam;
					if (s >= 0) {
						S_op(i, j) = 1;
						S_on(i, j) = (int)seams[s]->val;
						S_fx(i, j) = (int)seams[s]->is_fixed;
					}
				}
			}
		}

		std::ofstream file(filename);

		// Write Mesh
		file << V.rows() << " " << V.cols() << "\n";
		for (int i = 0; i < V.rows(); ++i) {
			file << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << "\n";
		}
		file << F.rows() << " " << F.cols() << "\n";
		for (int i = 0; i < F.rows(); ++i) {
			file << F(i, 0) << " " << F(i, 1) << " " << F(i, 2) << " " << F(i, 3) << "\n";
		}

		file << F_sh.size() << "\n";
		for (int i = 0; i < F_sh.size(); ++i) {
			file << F_sh(i) << "\n";
		}

		file << F_sr.size() << "\n";
		for (int i = 0; i < F_sr.size(); ++i) {
			file << F_sr(i) << "\n";
		}

		file << F_tx.size() << "\n";
		for (int i = 0; i < F_tx.size(); ++i) {
			file << F_tx(i) << "\n";
		}

		file << O_yl.rows() << " " << O_yl.cols() << "\n";
		for (int i = 0; i < O_yl.rows(); ++i) {
			file << O_yl(i, 0) << " " << O_yl(i, 1) << " " << O_yl(i, 2) << " " << O_yl(i, 3) << "\n";
		}

		file << O_io.rows() << " " << O_io.cols() << "\n";
		for (int i = 0; i < O_io.rows(); ++i) {
			file << O_io(i, 0) << " " << O_io(i, 1) << " " << O_io(i, 2) << " " << O_io(i, 3) << "\n";
		}

		file << O_fx.rows() << " " << O_fx.cols() << "\n";
		for (int i = 0; i < O_fx.rows(); ++i) {
			file << O_fx(i, 0) << " " << O_fx(i, 1) << " " << O_fx(i, 2) << " " << O_fx(i, 3) << "\n";
		}

		file << S_op.rows() << " " << S_op.cols() << "\n";
		for (int i = 0; i < S_op.rows(); ++i) {
			file << S_op(i, 0) << " " << S_op(i, 1) << " " << S_op(i, 2) << " " << S_op(i, 3) << "\n";
		}

		file << S_on.rows() << " " << S_on.cols() << "\n";
		for (int i = 0; i < S_on.rows(); ++i) {
			file << S_on(i, 0) << " " << S_on(i, 1) << " " << S_on(i, 2) << " " << S_on(i, 3) << "\n";
		}

		file << S_fx.rows() << " " << S_fx.cols() << "\n";
		for (int i = 0; i < S_fx.rows(); ++i) {
			file << S_fx(i, 0) << " " << S_fx(i, 1) << " " << S_fx(i, 2) << " " << S_fx(i, 3) << "\n";
		}

		file.close();
	}

}