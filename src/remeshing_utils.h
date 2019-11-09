#pragma once

#include <vector>

#include <Eigen/Core>

namespace hlk {

struct TM_Node {
    int index;
    int g_n_0;
    int g_n_1;
    Eigen::Vector3d m;
    Eigen::Vector3d n;
    Eigen::Vector3d d;
    double e;
    double q;
    bool b; // indicates whether it is on boundary
    TM_Node(int index_, int g_n_0_, int g_n_1_, double e_, double q_, bool b_, Eigen::Vector3d m_, Eigen::Vector3d n_, Eigen::Vector3d d_)
        : index(index_), g_n_0(g_n_0_), g_n_1(g_n_1_), e(e_), q(q_), b(b_), m(m_), n(n_), d(d_) {};
};

struct TM_Edge {
    int n_0 = -1;
    int n_1 = -1;
    double w = 0.0;
    TM_Edge(int n_0_, int n_1_, double w_)
        : n_0(n_0_), n_1(n_1_), w(w_) {};
};

struct PolyPoint {
    Eigen::Vector3d field_x, field_y;
};

struct FaceVector {
    int face_id = -1;
    bool is_hard = false;
    
    std::vector<bool> assigned; // size 2, using stl for serialization
    std::vector<Eigen::Vector3d> frame; // size 2, using stl for serialization
	Eigen::Vector3d base_vector; // perpendicular to frame[1]

    Eigen::Vector3d center;
    Eigen::Vector3d normal;
};

struct SplitEdge {
    int index_0;
    int index_1;
    Eigen::Vector3d normal;
};

enum ViewingMode {
    MESH_ONLY,
    MESH_CURL,
    MESH_FIELD,
    MESH_QUAD,
    QUAD_ONLY
};

enum DrawingMode {
    COURSE,
    WALE
};

enum MIQMode {
    CROSS,
    POLYVECTOR
};

struct CTRLZSL {
	bool ctrl = false;
	bool z = false;
	bool s = false;
	bool l = false;
};

struct TEMPDATA {
	std::string mesh;
	std::string face;
	std::string edge;
};

double angle_between(Eigen::Vector3d v1, Eigen::Vector3d v2);
double angle_coordinate_system(const Eigen::Vector3d& v, const Eigen::Vector3d& x, const Eigen::Vector3d& y);

void graph_dijkstra(std::vector<std::vector<double>>& graph, int src, int dest, std::vector<int>& path);
void graph_dijkstra(
    std::vector<TM_Node>& loop_gi_nodes,
    std::vector<std::vector<Eigen::Vector3d>>& graph,
    std::vector<bool>& graph_bound,
    int src, int dest, std::vector<int>& path);

double get_total_length(const std::vector<Eigen::Vector3d> & input_points);

bool is_almost_zero(double value);
bool is_almost_zero(float value);

int row_index_of(std::vector<std::vector<int>> arr2d, int target);

void line_texture(
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& texture_R,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& texture_G,
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& texture_B,
    bool add_stitches = false);

Eigen::Vector3d plane_project(
    Eigen::Vector3d planar_location, 
	Eigen::Vector3d planar_direction, 
    Eigen::Vector3d p);

std::vector<Eigen::Vector3d> UniformSampling(
	const std::vector<Eigen::Vector3d>& input_points, 
	const int sample_nb);

}

#include <igl/serialize.h>

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
