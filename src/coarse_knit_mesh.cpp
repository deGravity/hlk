#include "coarse_knit_mesh.h"

#include "glyphs.h"
#include "glyph.h"

namespace hlk {

    std::string nth_label(std::string label, int n) {
        return (label + "_" + std::to_string(n));
    }

    void CoarseKnitMesh::update_textures()
    {
        for (auto& edge : edges) {
            edge.update_texture();
        }
        for (auto& side : sides) {
            side.update_texture();
        }
        for (auto& quad : quads) {
            quad.update_texture();
        }
    }

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

        // Find singularity connecting edges and add them to seams
        // TODO - We can probably split these at intersections
        // is it worth doing?
        for (int sv : singular_vertices) {
            for (int side : out_sides(sv)) {
                auto separatrix = side_loop(side);
                if (is_singularity[side_v(separatrix.back())]) {
                    if (!is_boundary_side[side]) {
                        int sep_seam = edges[sides_to_edges[side]].seam;
                        if (sep_seam == -1) {
                            sep_seam = seams.size();
                            seams.emplace_back(geometry_optimizer.get_bool_prop(nth_label("seam", sep_seam)));
                        }
                        for (int seam_side : separatrix) {
                            edges[sides_to_edges[seam_side]].seam = sep_seam;
                        }
                    }
                }
            }
        }

        // Make Everything Invisible to start
        for (auto& slot : slots) {
            set_glyph(slot, glyphs::NONE, color::INVISIBLE);
        }

        // White Quad Background and border lines
        // (Should really be displaying the underlying mesh for this)
        /*
        for (auto& slot : quad_slots) {
            set_glyph(slot, glyphs::SOLID_LINE, color::WHITE);
        }
        */

        for (int i = 0; i < 4 * m; ++i) {
            if (is_boundary_side[i]) {
                set_glyph(half_edge_slots[i], glyphs::THIN_SOLID_LINE, color::GREY);
            }
        }

        for (auto& slot : edge_slots) {
            set_glyph(slot, glyphs::THIN_SOLID_LINE, color::GREY);
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
    void CoarseKnitQuad::update_texture()
    {
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
        if (is_regular) {
            
            int prev_idx = mesh->prev_side(index);
            auto& prev_is_loop = mesh->sides[prev_idx].is_loop;
            auto& prev_is_out = mesh->sides[prev_idx].is_out;

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
            // TODO - Do we need to count the effective degree of each
            // singular vertex?
            // Are there extra constraints here (like we can at most skip one
            // position in the ordering, or at most one in a row?)
            // If the label is completely the same, then we have effectively added
            // a vertex (the quad could be thought of as split)
            // If the label is the same direction but opposite orientation, then
            // we have effectively removed a vertex (merging quads).
            // In reality, performing either of these operations would create
            // T-Junctions in the mesh
            // We may want to constrain that singular corners are either normal,
            // or in one of these two cases. The other cases would reverse the
            // handedness. This may not be necessary, however, since the rest
            // of the mesh might enforce consistency. If we strip down to just
            // a singularity graph, however, these extra constraints would definitely
            // be warranted.
            // There may also be a desired constraint on the types of edges at which
            // we allow merging and splitting due to the T-Junctions that they
            // will create.
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