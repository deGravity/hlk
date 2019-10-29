#include "extract_quad_mesh.h"

#include <qex.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <igl/writeOBJ.h>

#include "read_quad_mesh.h"

namespace hlk {

	int QExCMD(std::string inputfile, std::string outputfile) {

		using namespace QEx;

		/*
		 * Read input mesh.
		 */
		QEx::TriMesh inputMesh;
		inputMesh.request_halfedge_texcoords2D();
		OpenMesh::IO::Options readOpts(OpenMesh::IO::Options::FaceTexCoord);
		OpenMesh::IO::read_mesh(inputMesh, inputfile, readOpts);

		/*
		 * Convert texture coordinates into separate uv vector.
		 */
		std::vector<OpenMesh::Vec2d> uvVector;
		uvVector.reserve(inputMesh.n_halfedges());
		for (TriMesh::HalfedgeIter he_it = inputMesh.halfedges_begin(), he_end = inputMesh.halfedges_end();
			he_it != he_end; ++he_it) {
			const OpenMesh::Vec2f& uv_f = inputMesh.texcoord2D(*he_it);
			OpenMesh::Vec2d uv(uv_f[0], uv_f[1]);
			uvVector.push_back(uv);
		}

		QEx::QuadMesh out;
		extractQuadMeshOMT(&inputMesh, &uvVector, 0, &out);

		/*
		 * Write output mesh.
		 */
		OpenMesh::IO::write_mesh(out, outputfile, OpenMesh::IO::Options::Default, 12);

		return 0;
	}

	void extract_quad_mesh(
		const Eigen::MatrixXd& V,
		const Eigen::MatrixXi& F,
		const Eigen::MatrixXd& TC,
		const Eigen::MatrixXi& FTC,
		QuadMesh& Q) {
		igl::writeOBJ("tmp_in.obj", V, F, Eigen::MatrixXd(), Eigen::MatrixXi(), TC, FTC);
		QExCMD("tmp_in.obj", "tmp_out.obj");
        read_quad_mesh("tmp_out.obj", Q);
	}
}