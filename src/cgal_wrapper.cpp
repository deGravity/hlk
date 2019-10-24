#include "cgal_wrapper.h"

namespace hlk {

bool CGAL_2D_Intersection_Segment_Segment(Point_2 s_0_s, Point_2 s_0_e, Point_2 s_1_s, Point_2 s_1_e, Point_2& inter) {
    if (is_almost_zero(std::sqrt(CGAL::squared_distance(s_0_s, Segment_2(s_1_s, s_1_e))))) {
        inter = s_0_s;
        return true;
    }
    if (is_almost_zero(std::sqrt(CGAL::squared_distance(s_0_e, Segment_2(s_1_s, s_1_e))))) {
        inter = s_0_e;
        return true;
    }
    if (is_almost_zero(std::sqrt(CGAL::squared_distance(s_1_s, Segment_2(s_0_s, s_0_e))))) {
        inter = s_1_s;
        return true;
    }
    if (is_almost_zero(std::sqrt(CGAL::squared_distance(s_1_e, Segment_2(s_0_s, s_0_e))))) {
        inter = s_1_e;
        return true;
    }

    CGAL::Object result = CGAL::intersection(Segment_2(s_0_s, s_0_e), Segment_2(s_1_s, s_1_e));
    if (const Point_2 *ipoint = CGAL::object_cast<Point_2>(&result)) {
        inter = *ipoint;
        return true;
    } else {
        return false;
    }
}

bool CGAL_2D_Intersection_Ray_Segment(Point_2 ray_s, Point_2 ray_e, Point_2 seg_s, Point_2 seg_e, Point_2& inter) {
	CGAL::Object result = CGAL::intersection(Ray_2(ray_s, ray_e), Segment_2(seg_s, seg_e));
	if (const Point_2 * ipoint = CGAL::object_cast<Point_2>(&result)) {
		inter = *ipoint;
		return true;
	} else {
		return false;
	}
}

Eigen::Vector3d CGAL_3D_Projection_Point_Segment(Point_3 p, Point_3 s_s, Point_3 s_e) {
    Line_3 l(s_s, s_e);
    Point_3 m_p = l.projection(p);

    double d_m_s = std::sqrt(CGAL::squared_distance(m_p, s_s));
    double d_m_e = std::sqrt(CGAL::squared_distance(m_p, s_e));
    double d_s_e = std::sqrt(CGAL::squared_distance(s_s, s_e));

    if (d_m_s >= d_s_e) {
        m_p = s_e;
    } else if (d_m_e >= d_s_e) {
        m_p = s_s;
    }
    return Eigen::Vector3d(m_p.x(), m_p.y(), m_p.z());
}

double CGAL_Distance_Point_Segments(const Eigen::Vector3d& p, const Polyline_type& polyline) {
	double min_diff = std::numeric_limits<double>::max();
	for (int i = 0; i < polyline.size() - 1; i++) {
		double dist = std::sqrt(CGAL::squared_distance(VectorPoint3d(p), Segment_3(polyline[i], polyline[i+1])));
        min_diff = min(min_diff, dist);
	}
	return min_diff;
}

double CGAL_Distance_Point_Segment(const Eigen::Vector3d& p, const Eigen::Vector3d& s, const Eigen::Vector3d& e){
	return std::sqrt(CGAL::squared_distance(VectorPoint3d(p), Segment_3(VectorPoint3d(s), VectorPoint3d(e))));
}

bool point_inside_triangle(Poly_facet_iterator& face, Eigen::Vector3d& p) {
    Point_3 p0 = face->halfedge()->next()->next()->vertex()->point();
    Point_3 p1 = face->halfedge()->vertex()->point();
    Point_3 p2 = face->halfedge()->next()->vertex()->point();
    Plane_3 plane(p1, CGAL::cross_product(p2 - p1, p0 - p1));
    Point_3 project = plane.projection(Point_3(p.x(), p.y(), p.z()));
    // Compute barycentric coordinates (u, v, w) for point p with respect to triangle (a, b, c).
    Vector_3 v0 = p1 - p0;
    Vector_3 v1 = p2 - p0;
    Vector_3 v2 = project - p0;
    double d00 = v0.squared_length();
    double d01 = v0 * v1;
    double d11 = v1.squared_length();
    double d20 = v2 * v0;
    double d21 = v2 * v1;
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0f - v - w;
    // check whether the point is inside the triangle
    return (u >= 0.0 && u <= 1.0) && (v >= 0.0 && v <= 1.0) && (w >= 0.0 && w <= 1.0);
}

bool detect_edge_point(const Point_and_primitive_id& pp, Halfedge_handle& handle, Eigen::Vector3d& n) {
    Point_3 p = pp.first;
    Poly_facet_iterator f = pp.second;

    Point_3 p0 = f->halfedge()->next()->next()->vertex()->point();
    Point_3 p1 = f->halfedge()->vertex()->point();
    Point_3 p2 = f->halfedge()->next()->vertex()->point();
    Vector_3 normal = CGAL::cross_product(p1 - p0, p2 - p0);
    n = Eigen::Vector3d(normal.x(), normal.y(), normal.z());

    double d0 = std::sqrt(CGAL::squared_distance(p, Segment_3(p0, p1))); // f->halfedge()
    double d1 = std::sqrt(CGAL::squared_distance(p, Segment_3(p1, p2))); // f->halfedge()->next()
    double d2 = std::sqrt(CGAL::squared_distance(p, Segment_3(p2, p0))); // f->halfedge()->next()->next()

    if (is_almost_zero(d0)) {
        handle = f->halfedge();
    } else if (is_almost_zero(d1)) {
        handle = f->halfedge()->next();
    } else if (is_almost_zero(d2)) {
        handle = f->halfedge()->next()->next();
    } else {
        return false;
    }
    return true;
}

bool first_intersection( Halfedge_handle& hh, int nb,
    Eigen::Vector3d inside, Eigen::Vector3d outside,
    Halfedge_handle& handle, Eigen::Vector3d& intersection) {

    Point_3 p0 = hh->next()->next()->vertex()->point();
    Point_3 p1 = hh->vertex()->point();
    Point_3 p2 = hh->next()->vertex()->point();
    Plane_3 plane(p1, CGAL::cross_product(p2 - p1, p0 - p1));

    Point_2 in2d = plane.to_2d(Point_3(inside.x(), inside.y(), inside.z()));
    Point_2 ou2d = plane.to_2d(Point_3(outside.x(), outside.y(), outside.z()));

    for (int i = 0; i < nb; i++) {
        hh = hh->next();
        Point_2 edge_0 = plane.to_2d(hh->vertex()->point());
        Point_2 edge_1 = plane.to_2d(hh->opposite()->vertex()->point());
        Point_2 intersection2d;
		if (CGAL_2D_Intersection_Ray_Segment(in2d, ou2d, edge_0, edge_1, intersection2d)) {
            Point_3 intersection3d = plane.to_3d(intersection2d);
            Point_3 edge_3d_0 = hh->vertex()->point();
            Point_3 edge_3d_1 = hh->opposite()->vertex()->point();
            intersection = CGAL_3D_Projection_Point_Segment(intersection3d, edge_3d_0, edge_3d_1);
            handle = hh;
			return true;
        }
    }
    return false;
}

int CGAL_Closest_Point(const Tree& tree, const Eigen::Vector3d& point) {

	Point_3 query(point.x(), point.y(), point.z());
	Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
	Point_3 p = pp.first;
	Poly_facet_iterator f = pp.second;

	int c_id = -1;
	double c_dist = std::numeric_limits<double>::max();
	for (int i = 0; i < 3; i++) {
		Halfedge_handle cur_handle=f->halfedge();
		for (int j = 0; j < i; j++)
			cur_handle = cur_handle->next();

		double dist = std::sqrt(CGAL::squared_distance(p, f->halfedge()->vertex()->point()));

		if (c_id < 0) {
			c_id = f->halfedge()->vertex()->id();
            c_dist = dist;
		} else {
			if (dist < c_dist) {
				c_id = f->halfedge()->vertex()->id();
                c_dist = dist;
			}
		}
	}

	return c_id;
}

int CGAL_Closest_Face(const Tree& tree, const Eigen::Vector3d& point) {
	Point_3 query(point.x(), point.y(), point.z());
	Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
	return  pp.second->id();
}

Eigen::Vector3d CGAL_Project(const Tree& tree, const Eigen::Vector3d& point) {
	Point_3 query(point.x(), point.y(), point.z());
	Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
	return  Point3dVector(pp.first);
}

std::vector<Eigen::Vector3d> CGAL_Mesh_Projection(
	const std::vector<Eigen::Vector3d>& features, const double insert_threshold,const Tree& tree) {

	std::vector<Eigen::Vector3d> igl_cutting_points;
	std::vector<Eigen::Vector3d> new_features;

	for (int i = 0; i < features.size(); i++) {
		Point_3 query(features[i].x(), features[i].y(), features[i].z());
		Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
		Halfedge_handle cur_handle;
		Eigen::Vector3d n;
		if (detect_edge_point(pp, cur_handle, n)) {
			Poly_point_3 p0 = cur_handle->vertex()->point();
			Poly_point_3 p1 = cur_handle->opposite()->vertex()->point();
			n = Eigen::Vector3d(p1.x() - p0.x(), p1.y() - p0.y(), p1.z() - p0.z()).cross(n);
			n *= (0.0005 / n.norm());
			new_features.push_back(features[i] + n);
		}
		else {
			new_features.push_back(features[i]);
		}
	}

	// related faces of projecting points
	std::vector<Poly_facet_iterator> project_faces;
	for (int i = 0; i < new_features.size(); i++) {
		Poly_point_3 query(new_features[i].x(), new_features[i].y(), new_features[i].z());
		Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
		project_faces.push_back(pp.second);
		//face_ids.push_back(pp.second->id());
	}

	// searching for all of the cutting points on edges
	int cur_face_id = project_faces[0]->id();
	Poly_facet_iterator cur_face = project_faces[0];
	Halfedge_handle cur_handle = cur_face->halfedge();
	Eigen::Vector3d inside = new_features[0];
	std::vector<Halfedge_handle> handles;
	int iteration = 0;
	int idx = 1;

	//TODO" there are still bugs here!!!!!
	while (true) {
		// std::cout << "[cgal remesh] " << iteration << "/" << new_features.size() << "\n";
		// search for the outside point of the current triangle
		bool goon = false;
		Halfedge_handle handle;
		Eigen::Vector3d intersection;
		while (true) {
			if (cur_face_id == project_faces[idx]->id()) {
				inside = new_features[idx];
				++idx;
				if (idx > new_features.size() - 1) break;
			}
			else {
				if (point_inside_triangle(cur_face, new_features[idx])) {
					++idx;
					if (idx > new_features.size() - 1) break;
				}
				else {

					if (first_intersection(cur_handle, (iteration == 0) ? 3 : 2, inside, new_features[idx], handle, intersection)) {
						goon = true;
						break;
					}
					else
					{
						++idx;
						if (idx > new_features.size() - 1) break;
					}
				}
			}
		}
		if (!goon) break;

		igl_cutting_points.push_back(intersection);
		handles.push_back(cur_handle);

		// move to next step
		inside = intersection;
		cur_handle = handle->opposite();
		cur_face = cur_handle->face();

		if (cur_face != NULL) {
			cur_face_id = cur_face->id();
			if (cur_face_id == project_faces[project_faces.size() - 1]->id()) break;
		}
		else { break; }

		++iteration;
	}

	if(!is_almost_zero((igl_cutting_points.front()- features.front()).norm()))
		igl_cutting_points.insert(igl_cutting_points.begin(), features.front());
	if (!is_almost_zero((igl_cutting_points.back() - features.back()).norm()))
		igl_cutting_points.emplace_back(features.back());

	return igl_cutting_points;
}

void CGAL_Mesh_Cutting(
    const std::vector<Eigen::Vector3d>& features, const double insert_threshold,
    const Tree& tree,
    std::vector<int>& igl_cutting_0_edges, std::vector<int>& igl_cutting_1_edges, 
    std::vector<Eigen::Vector3d>& igl_cutting_points,
    std::vector<std::vector<int>>& cutting_faces) {

    std::vector<Eigen::Vector3d> new_features;

    for (int i = 0; i < features.size(); i++) {
        Point_3 query(features[i].x(), features[i].y(), features[i].z());
        Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
        Halfedge_handle cur_handle;
        Eigen::Vector3d n;
        if (detect_edge_point(pp, cur_handle, n)) {
            Poly_point_3 p0 = cur_handle->vertex()->point();
            Poly_point_3 p1 = cur_handle->opposite()->vertex()->point();
            n = Eigen::Vector3d(p1.x() - p0.x(), p1.y() - p0.y(), p1.z() - p0.z()).cross(n);
            n *= (0.005 / n.norm());
            new_features.push_back(features[i] + n);
        } else {
            new_features.push_back(features[i]);
        }
    }

    // related faces of projecting points
    std::vector<Poly_facet_iterator> project_faces; 
    for (int i = 0; i < new_features.size(); i++) {
        Poly_point_3 query(new_features[i].x(), new_features[i].y(), new_features[i].z());
        Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
        project_faces.push_back(pp.second);
        //face_ids.push_back(pp.second->id());
    }

    // searching for all of the cutting points on edges
    int cur_face_id = project_faces[0]->id();
    Poly_facet_iterator cur_face = project_faces[0];
    Halfedge_handle cur_handle = cur_face->halfedge();
    Eigen::Vector3d inside = new_features[0];
    std::vector<Halfedge_handle> handles;
    int iteration = 0;
    int idx = 1;

	// TODO: there are still bugs here!!!!!
    while (true) {
        std::cout << "[cgal remesh] " << iteration << "/" << new_features.size() << "\n";
        // search for the outside point of the current triangle
        bool goon = false;
        while (true) {
            if (cur_face_id == project_faces[idx]->id()) {
                inside = new_features[idx];
                ++idx;
                if (idx > new_features.size() - 1) break;
            } else {
                if (point_inside_triangle(cur_face, new_features[idx])) {
                    ++idx;
                    if (idx > new_features.size() - 1) break;
                } else {
                    goon = true;
                    break;
                }
            }
        }
        if (!goon) break;

        Halfedge_handle handle;
        Eigen::Vector3d intersection;
        if (first_intersection(cur_handle, (iteration == 0) ? 3 : 2, inside, new_features[idx], handle, intersection)) {
			Point_3 edge_3d_0 = handle->vertex()->point();
			Point_3 edge_3d_1 = handle->opposite()->vertex()->point();
			double d0 = std::sqrt(CGAL::squared_distance(edge_3d_0, Point_3(intersection[0], intersection[1], intersection[2])));
			double d1 = std::sqrt(CGAL::squared_distance(edge_3d_1, Point_3(intersection[0], intersection[1], intersection[2])));
			if (d0 > insert_threshold && d1 > insert_threshold) {
				igl_cutting_points.push_back(intersection);
				handles.push_back(cur_handle);
			}
            // move to next step
            inside = intersection;
            cur_handle = handle->opposite();
            cur_face = cur_handle->face();
            if (cur_face != NULL) {
                cur_face_id = cur_face->id();
                if (cur_face_id == project_faces[project_faces.size() - 1]->id()) break;
            } else { break; }
        } else { break; }

        ++iteration;
    }
    
    for (int i = 0; i < handles.size(); i++) {
		int face_id = handles[i]->face()->id();
		int opposite_face_id = handles[i]->opposite()->face()->id();
        cutting_faces[face_id].push_back(i);
        cutting_faces[opposite_face_id].push_back(i);
        igl_cutting_0_edges.push_back(handles[i]->vertex()->id());
        igl_cutting_1_edges.push_back(handles[i]->opposite()->vertex()->id());
    }
}

Point_3 VectorPoint3d(Eigen::Vector3d p) {
	return Point_3(p[0], p[1], p[2]);
}

Eigen::Vector3d Point3dVector(Point_3 p) {
	return Eigen::Vector3d(p[0], p[1], p[2]);
}

Eigen::Vector3d CGAL_3D_Plane_Base_1(Eigen::Vector3d plane_p, Eigen::Vector3d plane_n) {
	Plane_3 plane(VectorPoint3d(plane_p), Vector_3(plane_n[0], plane_n[1], plane_n[2]));
	Vector_3 v = plane.base1();
	return Eigen::Vector3d(v[0], v[1], v[2]);
}

Eigen::Vector3d RotationAxis(Eigen::Vector3d p, double angle, Eigen::Vector3d n) {
	Eigen::Matrix4d inputMatrix;
	
	inputMatrix.row(0)[0] = p[0];
	inputMatrix.row(1)[0] = p[1];
	inputMatrix.row(2)[0] = p[2];
	inputMatrix.row(3)[0] = 1.0;
	double u = n[0];
	double v = n[1];
	double w = n[2];

	Eigen::Matrix4d  rotationMatrix;
	double L = (u * u + v * v + w * w);

	double u2 = u * u;
	double v2 = v * v;
	double w2 = w * w;

	rotationMatrix.row(0)[0] = (u2 + (v2 + w2) * cos(angle)) / L;
	rotationMatrix.row(0)[1] = (u * v * (1 - cos(angle)) - w * sqrt(L) * sin(angle)) / L;
	rotationMatrix.row(0)[2] = (u * w * (1 - cos(angle)) + v * sqrt(L) * sin(angle)) / L;
	rotationMatrix.row(0)[3] = 0.0;

	rotationMatrix.row(1)[0] = (u * v * (1 - cos(angle)) + w * sqrt(L) * sin(angle)) / L;
	rotationMatrix.row(1)[1] = (v2 + (u2 + w2) * cos(angle)) / L;
	rotationMatrix.row(1)[2] = (v * w * (1 - cos(angle)) - u * sqrt(L) * sin(angle)) / L;
	rotationMatrix.row(1)[3] = 0.0;

	rotationMatrix.row(2)[0] = (u * w * (1 - cos(angle)) - v * sqrt(L) * sin(angle)) / L;
	rotationMatrix.row(2)[1] = (v * w * (1 - cos(angle)) + u * sqrt(L) * sin(angle)) / L;
	rotationMatrix.row(2)[2] = (w2 + (u2 + v2) * cos(angle)) / L;
	rotationMatrix.row(2)[3] = 0.0;

	rotationMatrix.row(3)[0] = 0.0;
	rotationMatrix.row(3)[1] = 0.0;
	rotationMatrix.row(3)[2] = 0.0;
	rotationMatrix.row(3)[3] = 1.0;

	double outputMatrix[4][1] = { 0.0, 0.0, 0.0, 0.0 };

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 1; j++) {
			outputMatrix[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				outputMatrix[i][j] += rotationMatrix.row(i)[k] * inputMatrix.row(k)[j];
			}
		}
	}
	return Eigen::Vector3d(outputMatrix[0][0], outputMatrix[0][1], outputMatrix[0][2]);
}


void CGAL_Export_Segment(std::ofstream& export_file_output, int& export_index,
	std::string s_name, double r, double g, double b, Eigen::Vector3d start, Eigen::Vector3d end, double radius) {
	Eigen::Vector3d normal = end - start;
	Eigen::Vector3d base_1 = CGAL_3D_Plane_Base_1(start, normal);
	double length_base_1 = base_1.norm();

	base_1[0] = base_1[0] / length_base_1 * radius;
	base_1[1] = base_1[1] / length_base_1 * radius;
	base_1[2] = base_1[2] / length_base_1 * radius;

	std::vector<Eigen::Vector3d> vecs;

	for (int i = 0; i < 4; i++) {
		double angle = i * 2 * M_PI / 4;
		Eigen::Vector3d v = RotationAxis(normal + base_1, angle, normal);
		vecs.push_back(v + start);
	}
	for (int i = 0; i < 4; i++) {
		vecs.push_back(vecs[i] - normal);
	}

	std::vector<std::vector<int>> faces;

	int face_index_0[4] = { 0, 1, 2, 3 };
	int face_index_1[4] = { 5, 1, 0, 4 };
	int face_index_2[4] = { 4, 0, 3, 7 };
	int face_index_3[4] = { 5, 4, 7, 6 };
	int face_index_4[4] = { 7, 3, 2, 6 };
	int face_index_5[4] = { 6, 2, 1, 5 };

	faces.emplace_back(std::vector<int>(face_index_0, face_index_0 + 4));
	faces.emplace_back(std::vector<int>(face_index_1, face_index_1 + 4));
	faces.emplace_back(std::vector<int>(face_index_2, face_index_2 + 4));
	faces.emplace_back(std::vector<int>(face_index_3, face_index_3 + 4));
	faces.emplace_back(std::vector<int>(face_index_4, face_index_4 + 4));
	faces.emplace_back(std::vector<int>(face_index_5, face_index_5 + 4));

	export_file_output << "g " + s_name << std::endl;

	for (int i = 0; i < vecs.size(); i++) {
		export_file_output << "v " << vecs[i][0] << " " << vecs[i][1] << " " << vecs[i][2] << " " << r << " " << g << " " << b << std::endl;
	}

	for (int i = 0; i < faces.size(); i++) {
		export_file_output << "f ";
		for (int j = 0; j < faces[i].size(); j++) {
			export_file_output << faces[i][j] + export_index << " ";
		}
		export_file_output << "" << std::endl;
	}

	export_index += 8;
}


void CGAL_Export_Segments(std::string path, double r, double g, double b, double radius, const std::vector<Eigen::Vector3d>& points) {
	std::ofstream export_file_output(path);
	int export_index = 1;
	for (int i = 0; i < points.size() - 1; i++)
		CGAL_Export_Segment(export_file_output, export_index, "segments", r, g, b, points[i], points[i + 1], radius);
	export_file_output.clear();
	export_file_output.close();
}


void CGAL_Export_Segments(std::string path, double r, double g, double b, double radius, const std::vector<std::vector<Eigen::Vector3d>>& segments) {
	std::ofstream export_file_output(path);
	int export_index = 1;
	for(int i=0;i<segments.size();i++)
		for(int j=0;j<segments[i].size()-1;j++)
			CGAL_Export_Segment(export_file_output, export_index, "segments_"+std::to_string(i), r, g, b, segments[i][j], segments[i][j+1], radius);
	export_file_output.clear();
	export_file_output.close();
}

void CGAL_Plane_Cutting(const Polyhedron_3 &mesh, const Tree& tree, const Eigen::Vector3d& plane_p,
		const Eigen::Vector3d& plane_n, std::vector<Eigen::Vector3d>& loop_polyline) {

	Polylines loop_polylines;
	Polyline_type polyline;

	CGAL::Polygon_mesh_slicer<Polyhedron_3, K> slicer(mesh);
	slicer(K::Plane_3(VectorPoint3d(plane_p), Vector_3(plane_n[0], plane_n[1], plane_n[2])), std::back_inserter(loop_polylines));

	double min_diff = std::numeric_limits<double>::max();
	for (const auto& pl : loop_polylines) {
		double dis = CGAL_Distance_Point_Segments(plane_p, pl);
		if (dis < min_diff) {
			min_diff = dis;
			polyline = pl;
		}
	}

	if (is_almost_zero((Point3dVector(polyline[0]) - Point3dVector(polyline[polyline.size() - 1])).norm())) {
		polyline.erase(polyline.begin()+ polyline.size()-1);
		min_diff = std::numeric_limits<double>::max();
		int index = -1;
		for (int i = 0; i < polyline.size(); i++) {
			double dis= (Point3dVector(polyline[i]) - plane_p).norm();
			if (dis < min_diff) {
				min_diff = dis;
				index = i;
			}
		}

		if (index > 0) {
			for (int i = 0; i < index; i++) polyline.emplace_back(polyline[i]);
			polyline.erase(polyline.begin(), polyline.begin() + index);
		}
		polyline.emplace_back(polyline[0]);
	}

	loop_polyline.clear();
	for (const auto& p : polyline) loop_polyline.emplace_back(Point3dVector(p));
}

std::vector<Eigen::Vector3d> CGAL_Plane_Projection(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n) {
	K::Plane_3 plane(VectorPoint3d(plane_p), Vector_3(plane_n[0], plane_n[1], plane_n[2]));
	std::vector<Eigen::Vector3d> projections;
	for (int i = 0; i < points.size(); i++)
		projections.emplace_back(Point3dVector(plane.projection(VectorPoint3d(points[i]))));
	return projections;
}

Eigen::Vector3d CGAL_Plane_Projection(const Eigen::Vector3d& point, const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n) {
	K::Plane_3 plane(VectorPoint3d(plane_p), Vector_3(plane_n[0], plane_n[1], plane_n[2]));
	return Point3dVector(plane.projection(VectorPoint3d(point)));
}

void CGAL_Export_Point(std::ofstream& export_file_output, int& export_index,
	std::string s_name, double r, double g, double b, const Eigen::Vector3d &point, double radius) {

	std::vector<Eigen::Vector3d> vecs;
	vecs.push_back(Eigen::Vector3d(0.5, 0.5, 0.5));
	vecs.push_back(Eigen::Vector3d(-0.5, 0.5, 0.5));
	vecs.push_back(Eigen::Vector3d(-0.5, 0.5, -0.5));
	vecs.push_back(Eigen::Vector3d(0.5, 0.5, -0.5));
	vecs.push_back(Eigen::Vector3d(0.5, -0.5, 0.5));
	vecs.push_back(Eigen::Vector3d(-0.5, -0.5, 0.5));
	vecs.push_back(Eigen::Vector3d(-0.5, -0.5, -0.5));
	vecs.push_back(Eigen::Vector3d(0.5, -0.5, -0.5));

	int face_index_0[4] = { 0, 1, 2, 3 };
	int face_index_1[4] = { 5, 1, 0, 4 };
	int face_index_2[4] = { 4, 0, 3, 7 };
	int face_index_3[4] = { 5, 4, 7, 6 };
	int face_index_4[4] = { 7, 3, 2, 6 };
	int face_index_5[4] = { 6, 2, 1, 5 };

    std::vector<std::vector<int>> faces;
    faces.push_back(std::vector<int>(face_index_0, face_index_0 + 4));
	faces.push_back(std::vector<int>(face_index_1, face_index_1 + 4));
	faces.push_back(std::vector<int>(face_index_2, face_index_2 + 4));
	faces.push_back(std::vector<int>(face_index_3, face_index_3 + 4));
	faces.push_back(std::vector<int>(face_index_4, face_index_4 + 4));
	faces.push_back(std::vector<int>(face_index_5, face_index_5 + 4));

	export_file_output << "g " + s_name << std::endl;
	for (int i = 0; i < vecs.size(); i++) {
		vecs[i][0] = vecs[i][0] * radius + point[0];
		vecs[i][1] = vecs[i][1] * radius + point[1];
		vecs[i][2] = vecs[i][2] * radius + point[2];
		export_file_output << "v " << vecs[i][0] << " " << vecs[i][1] << " " << vecs[i][2] << " " << r << " " << g << " " << b << std::endl;
	}
	for (int i = 0; i < faces.size(); i++) {
		export_file_output << "f ";
		for (int j = faces[i].size() - 1; j >= 0; j--) {
			export_file_output << faces[i][j] + export_index << " ";
		}
		export_file_output << "" << std::endl;
	}
	export_index += 8;
}

void CGAL_Export_Points(std::string path, double r, double g, double b, double radius, const std::vector<Eigen::Vector3d>& points) {
	std::ofstream export_file_output(path);
	int export_index = 1;
	for (const auto& point : points)
		CGAL_Export_Point(export_file_output, export_index, "points", r, g, b, point, radius);
	export_file_output.clear();
	export_file_output.close();
}

void CGAL_Export_Points(std::string path, double r, double g, double b, double radius, const std::vector<std::vector<Eigen::Vector3d>>& pointses) {
	std::ofstream export_file_output(path);
	int export_index = 1;
	for(int i=0;i<pointses.size();i++)
		for (const auto& point : pointses[i])
		CGAL_Export_Point(export_file_output, export_index, "points_"+std::to_string(i), r, g, b, point, radius);
	export_file_output.clear();
	export_file_output.close();
}

}
