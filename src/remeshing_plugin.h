#pragma once

#include <string>
#include <vector>
#include <unordered_set>

#include <Eigen/Core>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include "cgal_wrapper.h"
#include "quad_mesh.h"
#include "symmetrizer.h"

namespace hlk {

class RemeshingMenu : public igl::opengl::glfw::imgui::ImGuiMenu {
public:
    RemeshingMenu(int nrosy, std::string input_path, std::string output_path) {
        plugin_name = "Remeshing";
        rosy = nrosy;
        in_path = input_path;
        out_path = output_path;

        click_threshold = 0.1f;
        soft_constraint_strength = 0.5f;
        gradient_size = 50.0f;
        stiffen_iter = 0;

        show_axis = false;
        show_stitches = false;
        symmetry_mode_yz = false;
        symmetry_mode_xz = false;
        symmetry_mode_xy = false;
        symmetrize_nrosy = false;
        has_direction_field = false;
        has_integer_grid = false;
        has_curl = false;
        is_quad_meshed = false;
        should_redraw = false;
        isInteger = true;

        geodesic_label = false;
        existing_edge_label = false;
        multi_points_drawing = true;

        viewing_mode = ViewingMode::MESH_ONLY;
        drawing_mode = DrawingMode::WALE;
        miq_mode = MIQMode::CROSS;
        cardinal = Cardinal::N;
        line_texture(texture_R, texture_G, texture_B);
    }
	~RemeshingMenu() {
		clear();
		for (auto& temp : temps) {
			remove(temp.mesh.c_str());
			remove(temp.face.c_str());
			remove(temp.edge.c_str());
		}
	};

    void init(igl::opengl::glfw::Viewer* _viewer);
    void draw_viewer_menu();
    bool load(std::string filename);
    bool save(std::string filename);
    void load_temp_data(std::string filename);
    void set_input_model(std::string filename) { input_model = filename; }

    /////////// CORE UI CALLBACKS ///////////
    bool mouse_down(int button, int modifier);
    bool mouse_move(int mouse_x, int mouse_y);
    bool mouse_up(int button, int modifier);
    bool mouse_scroll(float delta_y);

    //////////// UTILITY METHODS ////////////
    void clear();

    void setup_mesh();
    void get_mesh_information();
    bool model_loaded() { return V.rows() > 0 && F.rows() > 0; }
    void construct_half_edge(std::vector<int>& half_edges);
    void update_polyhedron_tree(const std::vector<int>& face_refs = std::vector<int>());
    void apply_subdivision();

    void assign_vector(int face_id, Eigen::Vector3d n);
    void assign_vector();
    std::unordered_set<int> neighbor_faces(std::vector<std::unordered_set<int>>& igl_faces, int e_0, int e_1);
    void geodesic_assign_vector();
    bool same_line(const std::vector<Eigen::Vector3d>& feature_points_save, const std::vector<Eigen::Vector3d>& feature_points);
    void symmetry_assign_vector(std::vector<int> axes, int start, int end);
    void symmetry_assign_vector(std::vector<int> axes, const std::vector<Eigen::Vector3d>& feature_points_save);
    void symmetry_assign_vector(const std::vector<Eigen::Vector3d>& feature_points_save);

    void split_mesh();
    void geodesic_split_mesh();
    void split_existing_edges(int index_0, int index_1, int insert_index);
    void symmetry_split_mesh(std::vector<int> axes, int start, int end);
    void symmetry_split_mesh(std::vector<int> axes, const std::vector<Eigen::Vector3d>& feature_points_save);
    void symmetry_split_mesh(const std::vector<Eigen::Vector3d>& feature_points_save);
    void cut_along_seams();

    void set_mesh_overlays(const int mesh_id, const bool wireframe = true, const bool overlay = true, const bool fill = true);
    void stylize_tri_mesh(const Eigen::MatrixXd& colors);
    void stylize_quad_mesh(const Eigen::MatrixXd& colors);
    void update_visualization();
    void update_drawing();
    void draw_direction_field();

	void draw_a_segment(const Eigen::Vector3d v0, const Eigen::Vector3d v1, const int color_index, const double face_dis = -1.);
	void draw_segments(const std::vector<Eigen::Vector3d>& segments, const int color_index, const double face_dis = -1.);
    void draw_a_point(const Eigen::Vector3d v, const int color_index, const double face_dis = -1.);
    void draw_points(const std::vector<Eigen::Vector3d>& vecs, const int color_index, const double face_dis = -1.);

    Symmetrizer symmetrizer;
    bool highlight_symmetries();
    std::vector<int> symmetry_axes();

    // field - impl in remeshing_field.cpp
    void reset_face_vectors();
    void reset_field();
    void init_curvature_field();
    void setup_boundary();
    void update_vectors_from_field(int direction = 1);
    void interpolate_cross_field(Eigen::VectorXd& S, int direction = 1); // default wale interpolation
    void interpolate_field();
    void generate_integer_grid();
    void init_curl();
    void reduce_curl();
    void quad_helix_finding();

    // loops - impl in remeshing_loops.cpp
    void clear_loops();
	void update_loop_graph();
	void compute_elastic_loop(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_v);
	void compute_elastic_loop_field_align(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);
	void compute_elastic_loop_min_geodesic(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);
	void symmetry_elastic_loop(const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);
	void symmetry_elastic_loop(std::vector<int> axes, const Eigen::Vector3d& plane_p, const Eigen::Vector3d& plane_n);

	void save_ctrlz();

    // bools...
    bool has_direction_field;
    bool has_curl;
    bool has_integer_grid;
    bool is_quad_meshed;
    bool should_redraw;
    bool isInteger;

    // numbers...
    float soft_constraint_strength;
    int stiffen_iter;
    float gradient_size;
    float yarn_size;
    float loop_size;
    float click_threshold; // a certain percentage of mesh edge size
    double mesh_size;
    double mesh_edge_size;
    Eigen::Vector3d mesh_center;

    // feature points
    std::vector<Eigen::Vector3d> feature_points;
    std::vector<int> feature_face_ids;

    // geodesic feature
    Eigen::Vector3d geodesic_point;
    std::vector<int> geodesic_path;
    int geodesic_index;
    bool geodesic_label;
    
    // directional faces
    std::vector<FaceVector> face_vectors;
    std::vector<SplitEdge> split_edges;
    std::vector<int> split_points;

	// loops data
	std::vector<TM_Node> loop_gi_nodes;
	std::vector<TM_Edge> loop_gi_edges;
	std::vector<std::unordered_set<int>> loop_g_iedges;
	std::vector<std::vector<Eigen::Vector3d>> loop_graph_adj;
	std::vector<bool> loop_graph_boundary;
	std::vector<Eigen::Vector3d> igl_v_ns;

	std::vector<Eigen::Vector3d> loop_points;
	std::vector<Eigen::Vector3d> loop_de_points;
	std::vector<std::vector<Eigen::Vector3d>> loop_polylines;
	std::vector<std::vector<Eigen::Vector3d>> loop_update_polylines;

	int loop_start_index = -1;
	int loop_end_index = -1;
	std::vector<Eigen::Vector3d> loop_path;
    std::vector<int> loop_feature_face_ids;

    // geometry data
    Polyhedron_3 igl_polyhedron;
    Tree igl_tree;
    std::vector<std::unordered_set<int>> igl_v_faces;
    std::vector<std::vector<double>> graph_adj;

    Eigen::MatrixXd direction_field[2];

    // curl reduction data
    Eigen::MatrixXi FField, FSings, FSeams;
    Eigen::MatrixXi EV, EF, FE;
    Eigen::MatrixXd VField, VSings, VSeams;
    Eigen::MatrixXd CField, CSings, CSeams;
    Eigen::MatrixXd rawField, combedField;
    Eigen::VectorXi matching, combedMatching;
    Eigen::VectorXd effort, combedEffort;
    Eigen::VectorXd curl; // norm of curl per edge
    Eigen::VectorXi singVertices, singIndices;
    Eigen::SparseMatrix<double> AE2F; // averaging curl to faces for visualization
    double curlMax, curlMaxOrig;
    Eigen::VectorXi c_b, c_blevel;
    Eigen::MatrixXd c_bc;

    // quad mesh data
    QuadMesh quad_mesh;
    // line textures
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> texture_R, texture_G, texture_B;

    // frame field data
    Eigen::MatrixXd V, B;
    Eigen::MatrixXi F;
    double global_scale;  // Scale for visualizing the fields
    // Input frame field constraints
    Eigen::VectorXi b;
    Eigen::MatrixXd bc1, bc2;
    // Interpolated frame field
    Eigen::MatrixXd FF1, FF2;
    // Deformed mesh
    Eigen::MatrixXd V_deformed, B_deformed;
    // Frame field on deformed
    Eigen::MatrixXd FF1_deformed, FF2_deformed;
    // Cross field on deformed
    Eigen::MatrixXd X1_deformed, X2_deformed;
    // Global parametrization
    Eigen::MatrixXd V_uv;
    Eigen::MatrixXi F_uv;
    // Local basis
    Eigen::MatrixXd B1, B2, B3;

    // polyvector field data
    Eigen::MatrixXcd polyvector_field;
    Eigen::VectorXi p_b;
    Eigen::MatrixXd p_bc;
    Eigen::MatrixXd VMeshCut;
    Eigen::MatrixXi FMeshCut;
    Eigen::MatrixXd cutUV;

    /////////////////// UI ///////////////////
    std::string in_path, out_path, input_model;
    ViewingMode viewing_mode;
    DrawingMode drawing_mode;
    MIQMode miq_mode;
    Cardinal cardinal;

    bool symmetry_mode_yz, symmetry_mode_xz, symmetry_mode_xy;
    bool symmetrize_nrosy;
    int rosy;

    bool show_axis, show_stitches, multi_points_drawing;

    bool existing_edge_label;

    int mouse_key;
    double mouse_x, mouse_y;
    bool ctrl_on, alt_on, shift_on, mouse_down_on;

	CTRLZSL czsl;
	std::vector<TEMPDATA> temps;
};

}
