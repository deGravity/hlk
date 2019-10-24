#include <igl/principal_curvature.h>
#include <igl/boundary_loop.h>
#include <igl/per_vertex_normals.h>

#include "remeshing_plugin.h"

namespace hlk {

void RemeshingPlugin::clear_loops() {
    loop_gi_nodes.clear();
    loop_gi_edges.clear();
    loop_g_iedges.clear();
    loop_graph_adj.clear();
    loop_graph_boundary.clear();
    igl_v_ns.clear();
    loop_points.clear();
    loop_de_points.clear();
    loop_polylines.clear();
    loop_update_polylines.clear();
    loop_start_index = -1;
    loop_end_index = -1;
    loop_path.clear();
}

void RemeshingPlugin::update_loop_graph() {
	auto build_underlying_edges = [&]() {
		int v_nb = viewer->data().V.rows();

		auto loop_g = std::vector<std::vector<bool>>(v_nb, std::vector<bool>(v_nb, false));
		auto igl_v_nbs = std::vector<std::unordered_set<int>>(v_nb, std::unordered_set<int>());
		for (int i = 0; i < viewer->data().F.rows(); i++) {
			int index_0 = viewer->data().F.row(i)[0];
			int index_1 = viewer->data().F.row(i)[1];
			int index_2 = viewer->data().F.row(i)[2];
			igl_v_nbs[index_0].emplace(index_1);
			igl_v_nbs[index_0].emplace(index_2);
			igl_v_nbs[index_1].emplace(index_0);
			igl_v_nbs[index_1].emplace(index_2);
			igl_v_nbs[index_2].emplace(index_0);
			igl_v_nbs[index_2].emplace(index_1);
		}

		for (int i = 0; i < igl_v_nbs.size(); i++) {
			std::vector<int> vs(1, i);
			//Breath first searching
			int vs_s(0), vs_e(vs.size());
			for (int iter = 0; iter < 1; iter++) {
				for (int j = vs_s; j < vs_e; j++) for (auto v : igl_v_nbs[vs[j]])
					if (std::find(vs.begin(), vs.end(), v) == vs.end()) {
						vs.emplace_back(v);
						if (!loop_g[i][v] && !loop_g[v][i])
							loop_g[i][v] = loop_g[v][i] = true;
					}
				vs_s = vs_e;
				vs_e = vs.size();
			}
		}
		return loop_g;
	};

	//parameters
	double alpha = 1.0;
	double beta = 0.25;

	//igl_v_ns
	Eigen::MatrixXd N;
	igl::per_vertex_normals(viewer->data().V, viewer->data().F, N);
	igl_v_ns.clear();
	for (int i = 0; i < viewer->data().V.rows(); i++)  igl_v_ns.emplace_back(N.row(i));

	//loop_g
	auto loop_g = build_underlying_edges();

	//curvature
	Eigen::MatrixXd ds_max, ds_min;
	Eigen::VectorXd ks_max, ks_min;
	igl::principal_curvature(viewer->data().V, viewer->data().F, ds_min, ds_max, ks_min, ks_max);

	//boundary
	std::vector<std::vector<int>> indices;
	igl::boundary_loop(viewer->data().F, indices);
	std::vector<bool> boundary_indices(viewer->data().V.rows(),false);
	for (auto& loop : indices) for (auto& indicate : loop) boundary_indices[indicate] = true;

#ifdef HAISEN
	int export_index = 1;
	int seg_nb = 0;
	std::ofstream o1("E:\\debug.obj");
#endif

	//loop_gi_nodes
	std::vector<TM_Node>().swap(loop_gi_nodes);
	auto igl_v_nbs = std::vector<std::vector<int>>(viewer->data().V.rows(), std::vector<int>());
	for (int i = 0; i < loop_g.size(); i++) {
		for (int j = 0; j < i; j++) {
			if (!loop_g[i][j]) continue;
			Eigen::Vector3d v_i = viewer->data().V.row(i);
			Eigen::Vector3d v_j = viewer->data().V.row(j);

			Eigen::Vector3d d_max_i = ds_max.row(i);
			Eigen::Vector3d d_max_j = ds_max.row(j);
			Eigen::Vector3d d_min_i = ds_min.row(i);
			Eigen::Vector3d d_min_j = ds_min.row(j);

			double k_max_i = ks_max.row(i)[0];
			double k_max_j = ks_max.row(j)[0];
			double k_min_i = ks_min.row(i)[0];
			double k_min_j = ks_min.row(j)[0];

			Eigen::Vector3d d_max = ((d_max_i + d_max_j) / 2.0).normalized();
			Eigen::Vector3d d_min = ((d_min_i + d_min_j) / 2.0).normalized();
			double k_max = (k_max_i + k_max_j) / 2.0;
			double k_min = (k_min_i + k_min_j) / 2.0;

			Eigen::Vector3d n_i = igl_v_ns[i];
			Eigen::Vector3d n_j = igl_v_ns[j];
			Eigen::Vector3d n = ((n_i + n_j) / 2.0).normalized();

			Eigen::Vector3d m = (v_i + v_j) / 2.0;
			Eigen::Vector3d d = (v_i - v_j).normalized();
			double e = (v_i - v_j).norm();
			int node_index = loop_gi_nodes.size();

			double q = std::pow(abs(k_min) - abs(k_max), 2.0) * (1 / (std::max(abs(d_min.dot(d)), abs(d_max.dot(d)))) - 1.0);
			
			bool b = boundary_indices[i] || boundary_indices[j];

			loop_gi_nodes.emplace_back(TM_Node(node_index, i, j, e, q, b, m, n, d));
			igl_v_nbs[i].emplace_back(node_index);
			igl_v_nbs[j].emplace_back(node_index);

#ifdef HAISEN
			CGAL_Export_Point(o1, export_index, 
				"node_" + std::to_string(node_index),
				0.5, 0.5, 0.5, m + n * 0.03, 0.03);
			seg_nb++;
#endif
		}
	}

	//boundary
	loop_graph_boundary = std::vector<bool>(loop_gi_nodes.size(), false);
	for (auto& node : loop_gi_nodes) loop_graph_boundary[node.index] = node.b;

	//loop_gi_edges
	std::vector<std::unordered_set<int>>().swap(loop_g_iedges);
	loop_g_iedges = std::vector<std::unordered_set<int>>(viewer->data().V.rows(), std::unordered_set<int>());
	loop_graph_adj = std::vector<std::vector<Eigen::Vector3d>>(loop_gi_nodes.size(), 
		std::vector<Eigen::Vector3d>(loop_gi_nodes.size(), Eigen::Vector3d(0.0,0.0,0.0)));

	std::vector<TM_Edge>().swap(loop_gi_edges);
	for (int i = 0; i < igl_v_nbs.size(); i++) {
		Eigen::Vector3d p = viewer->data().V.row(i);
		Eigen::Vector3d n = igl_v_ns[i];
		auto& nbs = igl_v_nbs[i];
		for (int j = 0; j < nbs.size(); j++) {
			auto& node_0 = loop_gi_nodes[nbs[j]];
			for (int k = j + 1; k < nbs.size(); k++) {
				auto& node_1 = loop_gi_nodes[nbs[k]];

				auto m_0 = plane_project(p, n, node_0.m);
				auto m_1 = plane_project(p, n, node_1.m);
				auto gamma = angle_between(p - m_0, m_1 - p);

				if (gamma < M_PI * 30.0 / 180.0) {
					//1:length
					double term_e= is_almost_zero(gamma)? (node_0.e + node_1.e) / 2.0:
						gamma * std::min(node_0.e, node_1.e) / (2.0* tan(gamma / 2.0)) + abs(node_0.e - node_1.e) / 2.0;

					term_e = (node_0.m - node_1.m).norm();

					//2:deviation from principal directions
					auto term_q = 0.5 * (node_0.q + node_1.q) * term_e;
					//term_q = 0.0;

					//3: geodesic curvature
					auto term_c = std::pow(gamma, 2.0) / std::min(node_0.e, node_1.e);
					term_c = 0.0;
					auto w = term_e + alpha * term_q + beta * term_c;
					loop_gi_edges.emplace_back(TM_Edge(node_0.index, node_1.index, w));
					loop_graph_adj[node_0.index][node_1.index] = Eigen::Vector3d(term_e,term_q,term_c);
					loop_graph_adj[node_1.index][node_0.index] = Eigen::Vector3d(term_e, term_q, term_c);
					loop_g_iedges[i].emplace(loop_gi_edges.size() - 1);
#ifdef HAISEN
					auto seg_name = "edge_" + std::to_string(node_0.index) + "_" + std::to_string(node_1.index) +"_"+ std::to_string(w);
					CGAL_Export_Segment(o1, export_index, seg_name, 0.5, 0.5, 0.5, 
						node_0.m+node_0.n*0.03, node_1.m+node_1.n* 0.03, 0.005);
					seg_nb++;
#endif
				}
			}
		}
	}

#ifdef HAISEN
	o1.clear();
	o1.close();
#endif
}

void RemeshingPlugin::symmetry_elastic_loop(std::vector<int> axes, const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n) {
	Eigen::Vector3d sym_plane_p = plane_p;
	Eigen::Vector3d sym_plane_n = sym_plane_p+plane_n;
	for (int axis : axes) {
		sym_plane_p[axis] = sym_plane_p[axis] + 2.0 * (mesh_center[axis] - sym_plane_p[axis]);
		sym_plane_n[axis] = sym_plane_n[axis] + 2.0 * (mesh_center[axis] - sym_plane_n[axis]);
	}

	sym_plane_n = sym_plane_n - sym_plane_p;

	if (!is_almost_zero((sym_plane_p - plane_p).norm())) compute_elastic_loop_min_geodesic(sym_plane_p, sym_plane_n);
}

void RemeshingPlugin::symmetry_elastic_loop(const Eigen::Vector3d & plane_p, const Eigen::Vector3d & plane_n) {
	//x
	if (symmetry_mode_yz && !symmetry_mode_xz && !symmetry_mode_xy) {
		symmetry_elastic_loop({ 0 }, plane_p, plane_n);
	}
	//y
	if (symmetry_mode_xz && !symmetry_mode_yz && !symmetry_mode_xy) {
		symmetry_elastic_loop({ 1 }, plane_p, plane_n);
	}
	//z
	if (symmetry_mode_xy && !symmetry_mode_yz && !symmetry_mode_xz) {
		symmetry_elastic_loop({ 2 }, plane_p, plane_n);
	}

	//x y
	if (symmetry_mode_yz && symmetry_mode_xz && !symmetry_mode_xy) {
		//x
		symmetry_elastic_loop({ 0 }, plane_p, plane_n);
		//y
		symmetry_elastic_loop({ 1 }, plane_p, plane_n);
		//x y
		symmetry_elastic_loop({ 0, 1 }, plane_p, plane_n);
	}
	//y z
	if (symmetry_mode_xz && symmetry_mode_xy && !symmetry_mode_yz) {
		//y
		symmetry_elastic_loop({ 1 }, plane_p, plane_n);
		//z
		symmetry_elastic_loop({ 2 }, plane_p, plane_n);
		//y z
		symmetry_elastic_loop({ 1, 2 }, plane_p, plane_n);
	}
	//x z
	if (symmetry_mode_yz && symmetry_mode_xy && !symmetry_mode_xz) {
		//x
		symmetry_elastic_loop({ 0 }, plane_p, plane_n);
		//z
		symmetry_elastic_loop({ 2 }, plane_p, plane_n);
		//x z
		symmetry_elastic_loop({ 0, 2 }, plane_p, plane_n);
	}

	//x y z
	if (symmetry_mode_yz && symmetry_mode_xy && symmetry_mode_xz) {
		//x
		symmetry_elastic_loop({ 0 }, plane_p, plane_n);
		//y
		symmetry_elastic_loop({ 1 }, plane_p, plane_n);
		//z
		symmetry_elastic_loop({ 2 }, plane_p, plane_n);
		//x y
		symmetry_elastic_loop({ 0, 1 }, plane_p, plane_n);
		//x z
		symmetry_elastic_loop({ 0, 2 }, plane_p, plane_n);
		//y z
		symmetry_elastic_loop({ 1, 2 }, plane_p, plane_n);
		//x y z
		symmetry_elastic_loop({ 0, 1, 2 }, plane_p, plane_n);
	}
}

void RemeshingPlugin::compute_elastic_loop(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_v) {
	
	std::cerr << "plane_p: " << plane_p[0] << "," << plane_p[1] << "," << plane_p[2] << std::endl;
	std::cerr << "plane_v: " << plane_v[0] << "," << plane_v[1] << "," << plane_v[2] << std::endl;

	int id = CGAL_Closest_Point(igl_tree, plane_p);

	double max_dif = std::numeric_limits<double>::min();
	int min_edge_index = -1;
	auto& iedges = loop_g_iedges[id];

	for (auto& edge_index : iedges) {
		auto& edge = loop_gi_edges[edge_index];
		auto& node_0 = loop_gi_nodes[edge.n_0];
		auto& node_1 = loop_gi_nodes[edge.n_1];

		Eigen::Vector3d p = viewer->data().V.row(id);
		Eigen::Vector3d n = igl_v_ns[id];
		
		auto m_0 = plane_project(p, n, node_0.m);
		auto m_1 = plane_project(p, n, node_1.m);
		
		double angle = angle_between(plane_v, m_0 - m_1);
		angle = angle / M_PI * 180.0;
		double dif = abs(plane_v.dot((m_0 - m_1).normalized()));

		if (min_edge_index < 0) {
			min_edge_index = edge_index;
			max_dif = dif;
		} else {
			if (dif > max_dif) {
				min_edge_index = edge_index;
				max_dif = dif;
			}
		}
	}

	auto& edge = loop_gi_edges[min_edge_index];
	auto& node_0 = loop_gi_nodes[edge.n_0];
	auto& node_1 = loop_gi_nodes[edge.n_1];

	loop_start_index = node_0.index;
	loop_end_index = node_1.index;

	std::cerr << "loop_start_index: " << loop_start_index << std::endl;
	std::cerr << "loop_end_index: " << loop_end_index << std::endl;

	auto save_w = loop_graph_adj[node_0.index][node_1.index];
	loop_graph_adj[node_0.index][node_1.index] = Eigen::Vector3d(0.0,0.0,0.0);
	loop_graph_adj[node_1.index][node_0.index] = Eigen::Vector3d(0.0, 0.0, 0.0);

	std::vector<int> loop_node_path;

	graph_dijkstra(loop_gi_nodes,loop_graph_adj, loop_graph_boundary, node_0.index, node_1.index, loop_node_path);

	if (loop_node_path.back() != node_1.index) {
		auto path = loop_node_path;
		graph_dijkstra(loop_gi_nodes, loop_graph_adj, loop_graph_boundary, node_1.index, node_0.index, loop_node_path);
		std::reverse(loop_node_path.begin(), loop_node_path.end());
		for (auto& indicate : path)loop_node_path.emplace_back(indicate);
	}

	loop_graph_adj[node_0.index][node_1.index] = save_w;
	loop_graph_adj[node_1.index][node_0.index] = save_w;

	//output
	std::cerr << "Loop: ";
	for (auto& p : loop_node_path) std::cerr << p << " / "; std::cerr << "" << std::endl;

	loop_path.clear();
	for (const auto& p : loop_node_path)
		loop_path.emplace_back(loop_gi_nodes[p].m);

	should_redraw = true;
}

void RemeshingPlugin::compute_elastic_loop_min_geodesic(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n) {
	std::cerr << "plane_p: " << plane_p[0] << "," << plane_p[1] << "," << plane_p[2] << std::endl;
	std::cerr << "plane_n: " << plane_n[0] << "," << plane_n[1] << "," << plane_n[2] << std::endl;

	auto one_iteration = [&](const std::vector<Eigen::Vector3d> & loop_polyline) {
		std::vector<int>loop_polyline_faces;
		for (auto p : loop_polyline)
			loop_polyline_faces.emplace_back(CGAL_Closest_Face(igl_tree, p));
		std::vector<Eigen::Vector3d> loop_update_polyline;
		for (int i = 0; i < loop_polyline.size(); i++) {
			if (i == 0 || i == loop_polyline.size() - 1) {
				loop_update_polyline.emplace_back(loop_polyline[i]);
			} else {
				Eigen::Vector3d pre_p = CGAL_Plane_Projection(loop_polyline[i - 1], face_vectors[loop_polyline_faces[i]].center, face_vectors[loop_polyline_faces[i]].normal);
				Eigen::Vector3d next_p = CGAL_Plane_Projection(loop_polyline[i + 1], face_vectors[loop_polyline_faces[i]].center, face_vectors[loop_polyline_faces[i]].normal);
				Eigen::Vector3d p = (pre_p+next_p)/2.0;
				loop_update_polyline.emplace_back(CGAL_Project(igl_tree, p));
			}
		}
		return loop_update_polyline;
	};

	auto terminal_condition = [](const std::vector<double> & errors, const int error_int, const double error_double) {
		if (errors.size() < error_int)return -1.0;
		double error_variations = 0.0;
		for (int i = errors.size() - 1; i >= errors.size() - error_int + 1; i--)
			error_variations += abs(errors[i] - errors[i - 1]);
		return error_variations;
	};

	loop_points.emplace_back(plane_p);

	// plane cutting
	std::vector<Eigen::Vector3d> loop_polyline;
	CGAL_Plane_Cutting(igl_polyhedron, igl_tree, plane_p, plane_n, loop_polyline);
	loop_polylines.emplace_back(loop_polyline);

	if (loop_polyline.size() < 2) return;

	std::vector<Eigen::Vector3d> loop_update_polyline = loop_polyline;

	auto total_error = get_total_length(loop_update_polyline);
	std::cerr << "Optimization: " << 0 << " : " << total_error << " / " << loop_update_polyline.size() << std::endl;

	int iter = 0;
	std::vector<double> errors;
	std::vector<std::vector<Eigen::Vector3d>> anchorses;
	std::vector<std::vector<Eigen::Vector3d>> polylines;
	while (true) {
		loop_update_polyline = one_iteration(loop_update_polyline);

		polylines.emplace_back(loop_update_polyline);
		errors.emplace_back(get_total_length(loop_update_polyline));
		double terminal_double = terminal_condition(errors, 5, 0.0001);

		std::cerr << "Optimization: " << iter << " : " << errors.back() << " terminal<0.1: " << terminal_double << " loop_size: " << loop_update_polyline.size() << std::endl;

		iter++;
		//if (terminal_double > 0.0 && terminal_double < 0.005)break;
		if (iter > 30) break;
	}

	std::vector<double>::iterator result = std::min_element(std::begin(errors), std::end(errors));
	loop_update_polyline = polylines[std::distance(std::begin(errors), result)];
	loop_update_polyline = CGAL_Mesh_Projection(loop_update_polyline, mesh_edge_size * click_threshold, igl_tree);
	loop_update_polylines.emplace_back(loop_update_polyline);
}

void RemeshingPlugin::compute_elastic_loop_field_align(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n) {

	std::cerr << "plane_p: " << plane_p[0] << "," << plane_p[1] << "," << plane_p[2] << std::endl;
	std::cerr << "plane_n: " << plane_n[0] << "," << plane_n[1] << "," << plane_n[2] << std::endl;

	auto field_align_error = [](const std::vector<Eigen::Vector3d> & loop_polyline,
		const Tree & igl_tree, const std::vector<FaceVector> & face_vectors, const int& base_vector_dir, const bool& b_a) {
			
		double total_error = 0.0;

		//assign force directions
		for (int i = 0; i < loop_polyline.size() - 1; i++) {
			Eigen::Vector3d center = (loop_polyline[i + 1] + loop_polyline[i]) / 2.0;
			int face_id = CGAL_Closest_Face(igl_tree, center);

			Eigen::Vector3d tangent_dir = loop_polyline[i + 1] - loop_polyline[i];
			Eigen::Vector3d field_x = base_vector_dir * face_vectors[face_id].frame[1];
			Eigen::Vector3d field_y = base_vector_dir * face_vectors[face_id].base_vector;

			if (b_a) std::swap(field_x, field_y);

			field_x.normalized();

			tangent_dir = plane_project(face_vectors[face_id].center,
				face_vectors[face_id].normal, face_vectors[face_id].center + tangent_dir) - face_vectors[face_id].center;

			auto angle = angle_between(field_x, tangent_dir);
			auto field_force = field_x * (tangent_dir.norm() * cos(angle)) - tangent_dir;

			angle *= 180.0 / M_PI;
			total_error += angle;
		}

		total_error = total_error / (loop_polyline.size() - 1);
		return total_error;
	};

	auto get_field_align_init = [](const std::vector<Eigen::Vector3d> & loop_polyline,
		const Tree & igl_tree, const std::vector<FaceVector> & face_vectors, int& base_vector_dir, bool& b_a) {

		int face_id = CGAL_Closest_Face(igl_tree, loop_polyline[0]);
		double angle_0 = angle_between(loop_polyline[1] - loop_polyline[0], face_vectors[face_id].frame[1]);
		double angle_1 = angle_between(loop_polyline[1] - loop_polyline[0], face_vectors[face_id].base_vector);
		base_vector_dir = 1;
		if (angle_0 > M_PI / 2.0) angle_0 = M_PI - angle_0;
		if (angle_1 > M_PI / 2.0) { angle_1 = M_PI - angle_1; base_vector_dir = -1; }
		b_a = angle_0 > angle_1;
	};

	auto get_field_align_force = [&](const std::vector<Eigen::Vector3d>& loop_polyline, const int& base_vector_dir, const bool& b_a,
		std::vector<Eigen::Vector3d>& anchors) {

		std::vector<int>loop_polyline_faces;
		for (auto p : loop_polyline)
			loop_polyline_faces.emplace_back(CGAL_Closest_Face(igl_tree, p));

		//points
		std::vector<PolyPoint> points;
		for (int i = 0; i < loop_polyline.size(); i++) {
			PolyPoint p;
			p.field_x = base_vector_dir * face_vectors[loop_polyline_faces[i]].frame[1];
			p.field_y = base_vector_dir * face_vectors[loop_polyline_faces[i]].base_vector;
			p.field_x.normalize();
			p.field_y.normalize();
			if (b_a) std::swap(p.field_x, p.field_y);
			points.emplace_back(p);
		}

		//segs
		std::vector<int> peaks;
		double last_anlge = 0.0;
		for (int i = 0; i < loop_polyline.size() - 1; i++){
			auto tangent_dir = loop_polyline[i + 1] - loop_polyline[i];
			auto angle = angle_coordinate_system(tangent_dir, points[i].field_x, points[i].field_y);
			if (i != 0&&last_anlge * angle <= 0) peaks.emplace_back(i);
			last_anlge = angle;
		}

		//fixed_anchors
		std::vector<int> fixed_anchors;
		if (peaks.size() >= 2) {
			for (int i = 0; i < peaks.size() - 1; i++)
				fixed_anchors.emplace_back((peaks[i] + peaks[i + 1]) / 2.0);
		}

		if (fixed_anchors.empty()) {
			fixed_anchors.emplace_back(0);
			fixed_anchors.emplace_back(loop_polyline.size() - 1);
		} else {
			if (fixed_anchors.front() != 0) fixed_anchors.insert(fixed_anchors.begin(), 0);
			if (fixed_anchors.back() != loop_polyline.size() - 1) fixed_anchors.emplace_back(loop_polyline.size() - 1);
		}

		for (auto anchor : fixed_anchors)
			anchors.emplace_back(loop_polyline[anchor]);

		//update_line
		std::vector<Eigen::Vector3d> update_line(1,loop_polyline.front());
		for (int i = 0; i < fixed_anchors.size() - 1; i++) {
			int anchor_s = fixed_anchors[i];
			int anchor_e = fixed_anchors[i+1];

			Eigen::Vector3d plane_s_p = loop_polyline[anchor_s];
			Eigen::Vector3d plane_e_p = loop_polyline[anchor_e];
			Eigen::Vector3d plane_s_n = points[anchor_s].field_x.cross(face_vectors[loop_polyline_faces[anchor_s]].normal).normalized();
			Eigen::Vector3d plane_e_n = points[anchor_e].field_x.cross(face_vectors[loop_polyline_faces[anchor_e]].normal).normalized();

			std::vector<Eigen::Vector3d> sub_line;
			for (int j = anchor_s; j <= anchor_e; j++) sub_line.emplace_back(loop_polyline[j]);

			std::vector<Eigen::Vector3d> projects_s = CGAL_Plane_Projection(sub_line, plane_s_p, plane_s_n);
			std::vector<Eigen::Vector3d> projects_e = CGAL_Plane_Projection(sub_line, plane_e_p, plane_e_n);

			double total_length = get_total_length(sub_line);
			double length = 0.0;
			for (int j = 1; j < sub_line.size(); j++) {
				double l = (sub_line[j] - sub_line[j - 1]).norm();
				length += l;
				update_line.emplace_back((1.0 - length / total_length)*projects_s[j] + (length/total_length)*projects_e[j]);
			}
		}

		//anchors

		//projection
		std::vector<Eigen::Vector3d> loop_update_polyline;
		for(auto p:update_line)
			loop_update_polyline.emplace_back(CGAL_Project(igl_tree, p));

		return loop_update_polyline;
	};

	auto terminal_condition = [](const std::vector<double> & errors, const int error_int,const double error_double) {
		if (errors.size() < error_int) return -1.0;
		double error_variations = 0.0;
		for (int i = errors.size() - 1; i >= errors.size() - error_int + 1; i--)
			error_variations += abs(errors[i] - errors[i - 1]);
		return error_variations;
	};

	loop_points.emplace_back(plane_p);

	//plane cutting
	std::vector<Eigen::Vector3d> loop_polyline;
	CGAL_Plane_Cutting(igl_polyhedron, igl_tree, plane_p, plane_n, loop_polyline);
	loop_polylines.emplace_back(loop_polyline);

	if (loop_polyline.size() < 2)return;

	int base_vector_dir;
	bool b_a;
	get_field_align_init(loop_polyline, igl_tree, face_vectors, base_vector_dir, b_a);

	std::vector<Eigen::Vector3d> loop_update_polyline = loop_polyline;

	auto total_error = field_align_error(loop_update_polyline, igl_tree, face_vectors, base_vector_dir, b_a);
	std::cerr << "Optimization: " << 0 << " : " << total_error << " / " << loop_update_polyline.size() << std::endl;

	int iter = 0;
	std::vector<double> errors;
	std::vector<std::vector<Eigen::Vector3d>> anchorses;
	std::vector<std::vector<Eigen::Vector3d>> polylines;
	while (true) {
		std::vector<Eigen::Vector3d> anchors;
		loop_update_polyline = get_field_align_force(loop_update_polyline, base_vector_dir, b_a, anchors);
		anchorses.emplace_back(anchors);

		polylines.emplace_back(loop_update_polyline);
		errors.emplace_back(field_align_error(loop_update_polyline, igl_tree, face_vectors, base_vector_dir, b_a));
		double terminal_double = terminal_condition(errors, 5, 0.1);
	
		std::cerr << "Optimization: " << iter << " : " << errors.back() <<" terminal<0.1: "<< terminal_double << " loop_size: " << loop_update_polyline.size() << std::endl;

		iter++;

		if (terminal_double>0.0&&terminal_double < 0.1)break;

		if (iter == 10)break;
	}

	std::vector<double>::iterator result = std::min_element(std::begin(errors), std::end(errors));
	loop_update_polyline = polylines[std::distance(std::begin(errors), result)];
	loop_update_polyline = CGAL_Mesh_Projection(loop_update_polyline, mesh_edge_size * click_threshold, igl_tree);
	loop_update_polylines.emplace_back(loop_update_polyline);
}

}
