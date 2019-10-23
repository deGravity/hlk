#include <igl/opengl/glfw/Viewer.h>
#include <igl/file_dialog_open.h>

#include "read_quad_mesh.h"
#include "labeled_quad_mesh.h"

//#include "optimizer.h"
#include <iostream>

#include <igl/png/readPNG.h>
#include "glyphs.h"
#include "glyph.h"
//#include <igl/opengl/create_shader_program.h>
//#include <igl/opengl/destroy_shader_program.h>

int main(void) {

	// Test Optimizer and Z3
	/*
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
	*/


	// Test transparent overlays
	igl::opengl::glfw::Viewer viewer;


	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
	igl::png::readPNG("glyphs.png", R, G, B, A);

	/*
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	V.resize(4, 3);
	V <<
		0, 0, 0,
		0, 1, 0,
		0, 1, 1,
		0, 0, 1;

	F.resize(2, 3);
	F <<
		0, 1, 2,
		0, 2, 3;

	Eigen::MatrixXd TC(4, 2);
	TC <<
		0, 0,
		1, 0,
		1, 1,
		0, 1;

	Eigen::VectorXi VTC(4);
	VTC << 0, 1, 2, 3;

	viewer.data().set_mesh(V, F);
	viewer.data().set_texture(R, G, B, A);
	viewer.data().set_uv(TC, VTC);
	viewer.data().show_lines = false;
	viewer.data().show_texture = true;
	viewer.data().set_colors(Eigen::RowVector4d(1.0, 1.0, 1.0, 1.0));

	*/

	// Test Quad Mesh structure
	
	hlk::LabeledQuadMesh Q;

	hlk::read_quad_mesh(igl::file_dialog_open(), Q);
	Q.init();
	
	viewer.data().set_mesh(Q.LV, Q.LF);
	viewer.data().set_mesh(Q.V, Q.F_t);

	for (int i = 0; i < Q.quad_slots.size(); ++i) {
		hlk::set_glyph(Q.LUV, Q.quad_slots[i], hlk::glyphs::SOLID_LINE, Eigen::RowVector4d(0.8, 0.2, 0.2, 1.0), Q.UV, Q.C);
		//hlk::set_glyph(Q.LUV, Q.quadrant_slot(i,0), hlk::glyphs::GEAR, Eigen::RowVector4d(0.0, 1.0, 0.0, 1.0), Q.UV, Q.C);
	}

	for (auto& slot : Q.vertex_slots) {
		hlk::set_glyph(Q.LUV, slot, hlk::glyphs::NONE, Eigen::RowVector4d(0.0, 0.0, 0.0, 0.0), Q.UV, Q.C);
	}

	hlk::set_glyph(Q.LUV, Q.quadrant_slots[0], hlk::glyphs::GEAR, Eigen::RowVector4d(0.0, 1.0, 0.0, 1.0), Q.UV, Q.C);
	hlk::set_glyph(Q.LUV, Q.quad_center_slot(3), hlk::glyphs::CIRCLE, Eigen::RowVector4d(0.0, 1.0, 1.0, 1.0), Q.UV, Q.C);

	hlk::set_glyph(Q.LUV, Q.half_edge_slots[9], hlk::glyphs::SEAM, Eigen::RowVector4d(0.5, .4, 1.0, 1.0), Q.UV, Q.C);

	hlk::set_glyph(Q.LUV, Q.dual_half_edge_slots[9], hlk::glyphs::SOLID_LINE, Eigen::RowVector4d(1, 1, 0, 1.0), Q.UV, Q.C);
	hlk::set_glyph(Q.LUV, Q.vertex_slots[3], hlk::glyphs::CIRCLE, Eigen::RowVector4d(0.0, 1.0, 1.0, 1.0), Q.UV, Q.C);

	hlk::set_glyph(Q.LUV, Q.edge_slots[2], hlk::glyphs::SPIRAL, Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), Q.UV, Q.C);

	/*
	double d = ((double)igl::FLOAT_EPS)*1000;
	Eigen::MatrixXd V(8, 3), UV(8, 2), C(8,4);
	Eigen::MatrixXi F(4, 3);
	V <<
		0, 0, 0,
		4, 0, 0,
		4, 4, 0,
		0, 4, 0,
		0, 0, d,
		2, 0, d,
		2, 2, d,
		0, 2, d;
	UV <<
		0.0, 0.83,
		0.9, 0.83,
		0.9, 0.874,
		0.0, 0.874,
		.5, 0,
		.74, 0,
		.74, .147,
		0, .147;
	F <<
		0, 1, 2,
		0, 2, 3,
		4, 5, 6,
		4, 6, 7;
	C <<
		1, 0, 0, 1,
		1, 0, 0, 1,
		1, 0, 0, 1,
		1, 0, 0, 1,
		0, 1, 0, 1,
		0, 1, 0, 1,
		0, 1, 0, 1,
		0, 1, 0, 1;

	*/

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

	/*

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
	
	*/

	return 0;
}