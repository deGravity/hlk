#include "remeshing_utils.h"

#include <igl/serialize.h>

// Single-TU emission of igl::serialization::_serialization for our types
// — the SERIALIZE_TYPE macro defines a non-inline free function, so it
// must only appear in one translation unit.
SERIALIZE_TYPE(hlk::FaceVector,
    SERIALIZE_MEMBER(face_id)
    SERIALIZE_MEMBER(is_hard)
    SERIALIZE_MEMBER(assigned)
    SERIALIZE_MEMBER(frame)
    SERIALIZE_MEMBER(base_vector)
    SERIALIZE_MEMBER(center)
    SERIALIZE_MEMBER(normal)
)

SERIALIZE_TYPE(hlk::SplitEdge,
    SERIALIZE_MEMBER(index_0)
    SERIALIZE_MEMBER(index_1)
    SERIALIZE_MEMBER(normal)
)

namespace hlk {

void get_edge_face_path(const std::string& filename, std::string& mesh_path_temp,
    std::string& face_path_temp, std::string& edge_path_temp, std::string& sing_path_temp) {

    std::size_t found = filename.find(".obj");
    if (found != std::string::npos) {
        face_path_temp = filename.substr(0, found) + "_temp.face";
        edge_path_temp = filename.substr(0, found) + "_temp.edge";
        sing_path_temp = filename.substr(0, found) + "_temp.sing";
        mesh_path_temp = filename;
    } else {
        face_path_temp = filename + "_temp.face";
        edge_path_temp = filename + "_temp.edge";
        sing_path_temp = filename + "_temp.sing";
        mesh_path_temp = filename + ".obj";
    }
};

double angle_between(Eigen::Vector3d v1,Eigen::Vector3d v2) {
    double d = v1.dot(v2) / (v1.norm() * v2.norm());
    if (is_almost_zero(d - 1.0)) { return 0.0; }
    if (is_almost_zero(d + 1.0)) { return M_PI; }
    return std::acos(d);
}

double angle_coordinate_system(const Eigen::Vector3d& v, const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
    double angle_x = angle_between(v, x);
    double angle_y = angle_between(v, y);

    if (angle_y > M_PI / 2.0) angle_x = -angle_x;

    return angle_x;
}

void graph_dijkstra(std::vector<std::vector<double>>& graph, int src, int dest, std::vector<int>& path) {

    auto graph_min_dist = [](std::vector<double> & dist, std::vector<bool> & visited) {
        double min = INT_MAX;
        int min_index = -1;
        for (int v = 0; v < dist.size(); v++) {
            if (!visited[v] && dist[v] <= min) {
                min = dist[v];
                min_index = v;
            }
        }
        return min_index;
    };

    path.clear();
    std::vector<int>().swap(path);

    int V = graph.size();
    std::vector<double> dist(V, INT_MAX);
    std::vector<bool> visited(V,false);
    std::vector<int> parent(V,-1);

    dist[src] = 0.0;
    for (int count = 0; count < V - 1; count++) {
        int u = graph_min_dist(dist, visited);
        visited[u] = true;

        if (u == dest) {
            int crawl = dest;
            path.push_back(crawl);
            while (parent[crawl] != -1) {
                path.push_back(parent[crawl]);
                crawl = parent[crawl];
            }
            if (path[path.size() - 1] != src) path.push_back(src);
            std::reverse(path.begin(), path.end());
            std::vector<double>().swap(dist);
            std::vector<bool>().swap(visited);
            std::vector<int>().swap(parent);
            return;
        }

        for (int v = 0; v < V; v++) {
            auto a = visited[v];
            auto b = graph[u][v];
            auto c = dist[u];
            auto d = graph[u][v];
            auto e = dist[v];

            if (!visited[v] && graph[u][v]
                && (dist[u] + graph[u][v] < dist[v])) {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }
}

void graph_dijkstra(
    std::vector<TM_Node>& loop_gi_nodes, 
    std::vector<std::vector<Eigen::Vector3d>>& graph, 
    std::vector<bool>& graph_bound, 
    int src, int dest, std::vector<int>& path) {

    auto graph_min_dist = [](std::vector<double>& dist, std::vector<bool>& visited) {
        double min = INT_MAX;
        int min_index = -1;
        for (int v = 0; v < dist.size(); v++) {
            if (!visited[v] && dist[v] <= min) {
                min = dist[v];
                min_index = v;
            }
        }
        return min_index;
    };

    //parameters
    double alpha = 0.1;
    double beta = 0.001;
    double gamma = 1.0 - alpha - beta;

    path.clear();
    std::vector<int>().swap(path);

    int V = graph.size();
    std::vector<double> dist(V, INT_MAX);
    std::vector<bool> visited(V, false);
    std::vector<int> parent(V, -1);

    dist[src] = 0.0;
    parent[src] = dest;
    for (int count = 0; count < V - 1; count++) {
        int u = graph_min_dist(dist, visited);
        visited[u] = true;

        if (graph_bound[u]) dest = u;

        if (u == dest) {
            int crawl = dest;
            path.push_back(crawl);
            while (true) {
                path.push_back(parent[crawl]);
                crawl = parent[crawl];
                if (crawl == src)break;
            }
            if (path[path.size() - 1] != src) path.push_back(src);
            std::reverse(path.begin(), path.end());
            std::vector<double>().swap(dist);
            std::vector<bool>().swap(visited);
            std::vector<int>().swap(parent);
            return;
        }

        for (int v = 0; v < V; v++) {
            //auto a = visited[v];
            //auto b = graph[u][v];
            //auto c = dist[u];
            //auto d = graph[u][v];
            //auto e = dist[v];
            auto term_e = graph[u][v][0];
            auto term_q = graph[u][v][1];
            auto term_c = graph[u][v][2];

            auto w = term_e + alpha * term_q + beta * term_c;

            if (!visited[v] && w) {
                auto& pre_node = loop_gi_nodes[parent[u]];
                auto& node = loop_gi_nodes[u];
                auto& next_node = loop_gi_nodes[v];

                //auto pre_m = pre_node.m;
                //auto m = node.m;
                //auto next_m = next_node.m;

                auto pre_m = plane_project(node.m, node.n, pre_node.m);
                auto m = node.m;
                auto next_m = plane_project(node.m, node.n, next_node.m);

                auto angle = angle_between(m - pre_m, next_m - m);
                term_c = std::pow(angle, 3.0) / std::min(pre_node.e, std::min(node.e, next_node.e));

                w = alpha * term_e + beta * term_q + gamma * term_c;
                //w = alpha*term_e + beta * term_c;

                if (dist[u] + w < dist[v]) {
                    parent[v] = u;
                    dist[v] = dist[u] + w;
                }
            }
        }
    }
}

bool is_almost_zero(double value) {
    return value < 1.0e-5 && value > -1.0e-5;
}

bool is_almost_zero(float value) {
    return value < 1.0e-5f && value > -1.0e-5f;
}

void line_texture(
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& texture_R,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& texture_G,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& texture_B,
    bool add_stitches) {

    unsigned size = 128;
    unsigned size2 = size / 2;
    unsigned lineWidth = 3;
    texture_R.setConstant(size, size, 255);

    if (add_stitches) {
        Eigen::Matrix<unsigned char, 8, 8> stitch_tex;
        stitch_tex <<
            255, 150, 255, 255, 255, 255, 150, 255,
            255, 150, 255, 255, 255, 255, 150, 255,
            150, 255, 150, 255, 255, 150, 255, 150,
            150, 255, 150, 255, 255, 150, 255, 150,
            255, 150, 255, 255, 255, 255, 150, 255,
            255, 150, 255, 255, 255, 255, 150, 255,
            150, 255, 150, 255, 255, 150, 255, 150,
            150, 255, 150, 255, 255, 150, 255, 150;
        for (int i = 0; i < 16; ++i) {
            for (int j = 0; j < 16; ++j) {
                texture_R.block<8, 8>(size - 1 - 8 * i, 8 * j) = stitch_tex.transpose();
            }
        }
    }

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < lineWidth; ++j) {
            texture_R(i, j) = 0;
            texture_R(i, size - 1 - j) = 0;
            texture_R(j, i) = 0;
            texture_R(size - 1 - j, 0) = 0;
        }
    }

    texture_G = texture_R;
    texture_B = texture_R;
}

int row_index_of(std::vector<std::vector<int>> arr2d, int target) {
    for (int i = 0; i < arr2d.size(); i++) {
        for (int j = 0; j < arr2d[i].size(); j++) {
            if (arr2d[i][j] == target) { return i; }
        }
    }
    return -1;
}

Eigen::Vector3d plane_project(Eigen::Vector3d planar_location, Eigen::Vector3d planar_direction, Eigen::Vector3d p) {
    if (is_almost_zero((planar_location-p).norm()))
        return planar_location;

    double angle = angle_between(planar_direction, p - planar_location);
    double length = (planar_location-p).norm();

    if (angle <= M_PI / 2.0)
        return p - planar_direction.normalized() * length * sin(M_PI / 2.0 - angle);
    else
        return p + planar_direction.normalized() * length * sin(angle - M_PI / 2.0);
}

double get_total_length(const std::vector<Eigen::Vector3d>& input_points) {
    double length = 0.0;
    if (input_points.size() >= 2)
        for (int i = 0; i < input_points.size() - 1; i++)
            length += (input_points[i] - input_points[i + 1]).norm();
    return length;
}

std::vector<Eigen::Vector3d> UniformSampling(const std::vector<Eigen::Vector3d>& input_points, const int sample_nb) {
    std::vector<Eigen::Vector3d> output_points;

    if (sample_nb <= 1) return output_points;
    double total_length = get_total_length(input_points);
    if (total_length <= 0.0) return output_points;
    double delta_length = total_length / (sample_nb-1);
    double length = 0.0;

    output_points.emplace_back(input_points.front());
    for (int i = 0; i < input_points.size() - 1; i++){

        double l = (input_points[i] - input_points[i + 1]).norm();
        double d = ((int)(length / delta_length) + 1) * delta_length - length;
        while (d >= 0 && d < l) {
            double ll = d / l;
            Eigen::Vector3d v = (double)(1.0 - ll) * input_points[i] + (double)(ll)* input_points[i + 1];
            output_points.push_back(v);
            d += delta_length;}
        length += l;
    }

    if(!is_almost_zero((output_points.back() - input_points.back()).norm()))
    output_points.emplace_back(input_points.back());

    return output_points;
}

}
