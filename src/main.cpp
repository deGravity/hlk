#include <iostream>

#include <igl/file_dialog_open.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/create_shader_program.h>
#include <igl/opengl/destroy_shader_program.h>
#include <igl/png/readPNG.h>

#include "glyphs.h"
#include "optimizer.h"
#include "quad_mesh.h"
#include "read_quad_mesh.h"
#include "remeshing_plugin.h"
#include "labeled_quad_mesh.h"
#include "glyph.h"

using namespace hlk;

void main_meshing() {	
	/////////////////////////////////////////////////////
	igl::opengl::glfw::Viewer viewer;
    int rosy = 4;
    std::string input_path = "./tmp_in.obj";
    std::string output_path = "./tmp_out.obj";
    std::string input_model = "";
	RemeshingPlugin remeshing_plugin(rosy, input_path, output_path);
	viewer.plugins.push_back((igl::opengl::glfw::ViewerPlugin *) &remeshing_plugin);
	if (!input_model.empty()) {
		remeshing_plugin.set_input_model(input_model);
	}
	viewer.launch();
	/////////////////////////////////////////////////////
}

void main_optimizer() {
	hlk::Optimizer opt;

	auto a = opt.get_bool_prop("a");
	auto b = opt.get_bool_prop("b");
	auto c = opt.get_bool_prop("c");

	opt.add_constraint(a && b);
	opt.add_constraint(b != c);

	auto result = opt.solve();

	if (result.has_result) {
		opt.update_all_props(*result.result_model);
	}

	std::cout << "a = " << a->val << " , b = " << b->val << " , c = " << c->val;
}


void main_debug_labeling() {
	igl::opengl::glfw::Viewer viewer;
	Eigen::MatrixXd V(8, 3), C(8, 4);
	Eigen::MatrixXi F(4, 3);
	V <<
		1, 1, 1,
		1, 1, -1,
		1, -1, 1,
		-1, 1, 1,

		2, 2, 2,
		2, 2, 2.5,
		2, 2, 1.5,
		2, 2.5, 2;
	F <<
		0, 2, 1,
		0, 1, 3,
		0, 3, 2,
		4, 6, 7,
	C <<
		1.0, 0.0, 0.0, 1.0,
		1.0, 0.0, 0.0, 1.0,
		1.0, 0.0, 0.0, 1.0,
		1.0, 0.0, 0.0, 1.0,

		0.0, 1.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0;

	viewer.data().set_mesh(V, F);
	viewer.data().set_colors(C);
	viewer.launch();
}

void main_labeling() {
	igl::opengl::glfw::Viewer viewer;

	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	igl::png::readPNG("glyphs.png", R, G, B, A);
	hlk::LabeledQuadMesh Q;
	hlk::read_quad_mesh(igl::file_dialog_open(), Q);

	for (auto& slot : Q.slots) {
		Q.set_glyph(slot, hlk::glyphs::NONE, hlk::color::INVISIBLE);
	}

	for (auto& slot : Q.quad_slots) {
		Q.set_glyph(slot, hlk::glyphs::SOLID_LINE, hlk::color::RED);
	}

	for (auto& slot : Q.quadrant_slots) {
		Q.set_glyph(slot, hlk::glyphs::GEAR, hlk::color::WHITE);
	}

	for (auto& slot : Q.edge_slots) {
		Q.set_glyph(slot, hlk::glyphs::SOLID_LINE, hlk::color::GREEN);
	}

	for (auto& slot : Q.dual_half_edge_slots) {
		Q.set_glyph(slot, hlk::glyphs::SOLID_ARROW, hlk::color::WHITE);
	}

	for (auto& slot : Q.half_edge_slots) {
		Q.set_glyph(slot, hlk::glyphs::DASHED_LINE, hlk::color::BLACK);
	}

	for (auto& slot : Q.vertex_slots) {
		Q.set_glyph(slot, hlk::glyphs::CIRCLE, hlk::color::BLUE);
	}

	std::vector<int> loop = Q.dual_loop(0, 0);
	for (int side : loop) {
		Q.set_glyph(
			Q.dual_half_edge_slots[side], // Where
			hlk::glyphs::THIN_SOLID_LINE, // Which Texture
			hlk::color::WHITE // What Color
		);
	}

	viewer.data().set_mesh(Q.LV, Q.LF);
	viewer.data().set_texture(R, G, B, A);
	viewer.data().set_uv(Q.UV);
	viewer.data().show_texture = true;
	viewer.data().show_lines = false;
	viewer.data().set_colors(Q.C);

	viewer.launch();


}

void main_metcap() {

	igl::opengl::glfw::Viewer viewer;


	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	igl::png::readPNG("jade.png", R, G, B, A);
	hlk::LabeledQuadMesh Q;

	hlk::read_quad_mesh(igl::file_dialog_open(), Q);
	Q.init();

	viewer.data().set_mesh(Q.V, Q.F_t);

	// Test met-cap: https://www.alecjacobson.com/weblog/?p=4827
	viewer.data().set_texture(R, G, B, A);
	viewer.data().set_face_based(false);
	viewer.data().show_lines = false;
	viewer.data().show_texture = true;
	viewer.launch_init(true, false);

	viewer.data().meshgl.init();
	igl::opengl::destroy_shader_program(viewer.data().meshgl.shader_mesh);

	{
		std::string mesh_vertex_shader_string =
			R"(#version 150
uniform mat4 view;
uniform mat4 proj;
uniform mat4 normal_matrix;
in vec3 position;
in vec3 normal;
out vec3 normal_eye;

void main()
{
  normal_eye = normalize(vec3 (normal_matrix * vec4 (normal, 0.0)));
  gl_Position = proj * view * vec4(position, 1.0);
})";

		std::string mesh_fragment_shader_string =
			R"(#version 150
in vec3 normal_eye;
out vec4 outColor;
uniform sampler2D tex;
void main()
{
  vec2 uv = normalize(normal_eye).xy * 0.5 + 0.5;
  outColor = texture(tex, uv);
})";

		igl::opengl::create_shader_program(
			mesh_vertex_shader_string,
			mesh_fragment_shader_string,
			{},
			viewer.data().meshgl.shader_mesh);
	}

	viewer.launch_rendering(true);
	viewer.launch_shut();

}

int main(void) {
	main_labeling();
	return 0;
}