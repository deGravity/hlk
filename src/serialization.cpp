#include "serialization.h"

namespace hlk {
	void save(std::ofstream& f, const Eigen::MatrixXd& M) {
		f << M.rows() << " " << M.cols() << std::endl;
		for (int i = 0; i < M.rows(); ++i) {
			for (int j = 0; j < M.cols(); ++j) {
				f << M(i, j);
				if (j < M.cols() - 1) {
					f << " ";
				}
			}
			f << std::endl;
		}
	}
	void load(std::ifstream& f, Eigen::MatrixXd& M) {
		int r, c;
		double val;
		f >> r >> c;
		M.resize(r, c);
		for (int i = 0; i < r; ++i) {
			for (int j = 0; j < c; ++j) {
				f >> val;
				M(i, j) = val;
			}
		}
	}

	void save(std::ofstream& f, const Eigen::MatrixXi& M) {
		f << M.rows() << " " << M.cols() << std::endl;
		for (int i = 0; i < M.rows(); ++i) {
			for (int j = 0; j < M.cols(); ++j) {
				f << M(i, j);
				if (j < M.cols() - 1) {
					f << " ";
				}
			}
			f << std::endl;
		}
	}
	void load(std::ifstream& f, Eigen::MatrixXi& M) {
		int r, c;
		int val;
		f >> r >> c;
		M.resize(r, c);
		for (int i = 0; i < r; ++i) {
			for (int j = 0; j < c; ++j) {
				f >> val;
				M(i, j) = val;
			}
		}
	}

	void save(std::ofstream& f, const Eigen::VectorXd& V) {
		f << V.size() << std::endl;
		for (int i = 0; i < V.size(); ++i) {
			f << V(i);
			if (i < V.size() - 1) {
				f << " ";
			}
		}
		f << std::endl;
	}
	void load(std::ifstream& f, Eigen::VectorXd& V) {
		int s;
		double val;
		f >> s;
		V.resize(s);
		for (int i = 0; i < s; ++i) {
			f >> val;
			V(i) = val;
		}
	}

	void save(std::ofstream& f, const Eigen::VectorXi& V) {
		f << V.size() << std::endl;
		for (int i = 0; i < V.size(); ++i) {
			f << V(i);
			if (i < V.size() - 1) {
				f << " ";
			}
		}
		f << std::endl;
	}

	void load(std::ifstream& f, Eigen::VectorXi& V) {
		int s;
		int val;
		f >> s;
		V.resize(s);
		for (int i = 0; i < s; ++i) {
			f >> val;
			V(i) = val;
		}
	}
}