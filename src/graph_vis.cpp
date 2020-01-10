#include "graph_vis.h"

namespace hlk {

	int GraphVisualization::append_row(Eigen::MatrixXd & M, const Eigen::RowVector3d& R)
	{
		int i = M.rows();
		M.conservativeResize(M.rows() + 1, M.cols());
		M.row(i) = R;
		return i;
	}

	int GraphVisualization::append_row(Eigen::MatrixXi & M, const Eigen::RowVector2i & R)
	{
		int i = M.rows();
		M.conservativeResize(M.rows() + 1, M.cols());
		M.row(i) = R;
		return i;
	}

	int GraphVisualization::append_row(Eigen::MatrixXi & M, int a, int b)
	{
		return append_row(M, Eigen::RowVector2i(a, b));
	}

	void GraphVisualization::add_point(int i, const Eigen::RowVector3d & c)
	{
		P.push_back(i);
		append_row(Pc, c);
	}

	void GraphVisualization::add_point(const Eigen::RowVector3d & p, const Eigen::RowVector3d & c)
	{
		int i = append_row(V, p);
		add_point(i, c);
	}

	void GraphVisualization::add_line(int a, int b, const Eigen::RowVector3d & c)
	{
		append_row(E, a, b);
		append_row(Ec, c);
	}

	void GraphVisualization::add_line(const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & c)
	{
		int ai = append_row(V, a);
		int bi = append_row(V, b);
		add_line(ai, bi, c);
	}

	void GraphVisualization::add_arrow(int a, int b, const Eigen::RowVector3d & n, const Eigen::RowVector3d & c)
	{
		append_row(A, a, b);
		append_row(An, n);
		append_row(Ac, c);
	}

	void GraphVisualization::add_arrow(const Eigen::RowVector3d & a, const Eigen::RowVector3d & b, const Eigen::RowVector3d & n, const Eigen::RowVector3d & c)
	{
		int ai = append_row(V, a);
		int bi = append_row(V, b);
		add_arrow(ai, bi, n, c);
	}

	void GraphVisualization::add_label(int i, const std::string & text)
	{
		L.push_back(text);
		Lp.push_back(i);
	}

	void GraphVisualization::add_label(const Eigen::RowVector3d & pos, const std::string & text)
	{
		int i = append_row(V, pos);
		add_label(i, text);
	}

	IGLVisualization GraphVisualization::get_vis()
	{
		IGLVisualization vis;

		
		vis.P.resize(P.size(), 3);
		for (int i = 0; i < P.size(); ++i) {
			vis.P.row(i) = V.row(P[i]);
		}

		
		vis.P_c = Pc;
		vis.V = V;
		vis.E = E;
		vis.E_c = Ec;
		vis.L = L;
		
		vis.L_p.resize(Lp.size(), 3);
		for (int i = 0; i < Lp.size(); ++i) {
			vis.L_p.row(i) = V.row(Lp[i]);
		}
		

		// Now Add Extra Edge Rows for the arrows and arrow heads
		
		for (int i = 0; i < A.rows(); ++i) {
			Eigen::RowVector3d a = V.row(A(i, 0));
			Eigen::RowVector3d b = V.row(A(i, 1));
			double len = (b - a).norm();
			double pct = spacing;
			double ahp = arrow_head_length;
			Eigen::RowVector3d dir = (b - a).normalized();
			Eigen::RowVector3d A = a + (1.0 - pct) / 2 * dir;
			Eigen::RowVector3d B = b - (1.0 - pct) / 2 * dir;
			Eigen::RowVector3d N = An.row(i);
			Eigen::RowVector3d X = dir.cross(N).normalized();
			Eigen::RowVector3d tip1 = B + X * len * ahp - dir * len * ahp;
			Eigen::RowVector3d tip2 = B - X * len * ahp - dir * len * ahp;
			Eigen::RowVector3d color = Ac.row(i);

			int ai = append_row(vis.V, A);
			int bi = append_row(vis.V, B);
			int t1i = append_row(vis.V, tip1);
			int t2i = append_row(vis.V, tip2);
			
			append_row(vis.E, ai, bi);
			append_row(vis.E, bi, t1i);
			append_row(vis.E, bi, t2i);
			
			for (int j = 0; j < 3; ++j) {
				append_row(vis.E_c, color);
			}
		}
		

		return vis;
	}

	void IGLVisualization::display(igl::opengl::ViewerData & viewer_data)
	{
		
		viewer_data.clear_labels();
		viewer_data.set_edges(V, E, E_c);
		viewer_data.set_points(P, P_c);
		for (int i = 0; i < L.size(); ++i) {
			viewer_data.add_label(L_p.row(i), L[i]);
		}
		
	}

}