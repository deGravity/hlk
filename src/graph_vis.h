#pragma once

#include <Eigen/Core>
#include <igl/opengl/ViewerData.h>
#include <vector>
#include <string>
namespace hlk {

	struct IGLVisualization {
		Eigen::MatrixXd P = Eigen::MatrixXd(0, 3);
		Eigen::MatrixXd P_c = Eigen::MatrixXd(0, 3);
		Eigen::MatrixXd V = Eigen::MatrixXd(0, 3);
		Eigen::MatrixXi E = Eigen::MatrixXi(0, 2);
		Eigen::MatrixXd E_c = Eigen::MatrixXd(0, 3);
		std::vector<std::string> L;
		Eigen::MatrixXd L_p = Eigen::MatrixXd(0, 3);

		void display(igl::opengl::ViewerData& viewer_data);

	};

	struct GraphVisualization {

		Eigen::MatrixXd V = Eigen::MatrixXd(0, 3);
		std::vector<int> P;
		Eigen::MatrixXd Pc = Eigen::MatrixXd(0, 3);
		Eigen::MatrixXi E = Eigen::MatrixXi(0, 2);
		Eigen::MatrixXd Ec = Eigen::MatrixXd(0, 3);
		Eigen::MatrixXi A = Eigen::MatrixXi(0, 2);
		Eigen::MatrixXd An = Eigen::MatrixXd(0, 3);
		Eigen::MatrixXd Ac = Eigen::MatrixXd(0, 3);
		std::vector<std::string> L;
		std::vector<int> Lp;
		
		// Helper to Append a row to an Eigen Matrix and return the row number
		// This should really be templatized and put in a utility header...
		static int append_row(Eigen::MatrixXd& M, const Eigen::RowVector3d& R);
		static int append_row(Eigen::MatrixXi& M, const Eigen::RowVector2i& R);
		static int append_row(Eigen::MatrixXi& M, int a, int b);


		double arrow_head_length;
		double spacing;

		void add_point(int i, const Eigen::RowVector3d& c);
		void add_point(const Eigen::RowVector3d& p, const Eigen::RowVector3d& c);

		void add_line(int a, int b, const Eigen::RowVector3d& c);
		void add_line(
			const Eigen::RowVector3d& a, 
			const Eigen::RowVector3d& b, 
			const Eigen::RowVector3d& c);

		void add_arrow(int a, int b, 
			const Eigen::RowVector3d& n, 
			const Eigen::RowVector3d& c);
		void add_arrow(
			const Eigen::RowVector3d& a,
			const Eigen::RowVector3d& b,
			const Eigen::RowVector3d& n,
			const Eigen::RowVector3d& c);

		void add_label(int i, const std::string& text);
		void add_label(const Eigen::RowVector3d& pos, const std::string& text);

		IGLVisualization get_vis();
	};

}