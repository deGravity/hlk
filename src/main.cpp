#include <igl/opengl/glfw/Viewer.h>
#include <igl/file_dialog_open.h>

#include "read_quad_mesh.h"
#include "quad_mesh.h"

#include "optimizer.h"
#include <iostream>

int main(void) {

	// Test Optimizer and Z3
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

	// Test Quad Mesh structure
	hlk::QuadMesh Q;

	hlk::read_quad_mesh(igl::file_dialog_open(), Q);

	igl::opengl::glfw::Viewer viewer;

	viewer.data().set_mesh(Q.V, Q.F_t);

	viewer.launch();
	
}