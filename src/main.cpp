#include <igl/opengl/glfw/Viewer.h>
#include <igl/file_dialog_open.h>

#include "read_quad_mesh.h"
#include "quad_mesh.h"

#include "optimizer.h"
#include <iostream>

#include <igl/png/readPNG.h>
#include "glyphs.h"
#include <igl/opengl/create_shader_program.h>
#include <igl/opengl/destroy_shader_program.h>

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
	igl::png::readPNG("ceramic.png", R, G, B, A);

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
	
	hlk::QuadMesh Q;

	hlk::read_quad_mesh(igl::file_dialog_open(), Q);
	

	viewer.data().set_mesh(Q.V, Q.F_t);
	
	//viewer.launch();

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