#include <igl/opengl/glfw/Viewer.h>
#include <igl/file_dialog_open.h>

#include "read_quad_mesh.h"
#include "quad_mesh.h"

int main(void) {
	hlk::QuadMesh Q;

	hlk::read_quad_mesh(igl::file_dialog_open(), Q);

	igl::opengl::glfw::Viewer viewer;

	viewer.data().set_mesh(Q.V, Q.F_t);

	viewer.launch();
}