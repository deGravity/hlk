#pragma once

#include <fstream>
#include <Eigen/Core>
#include <vector>
#include <utility>

namespace hlk {
	void save(std::ofstream& f, const Eigen::MatrixXd& M);
	void load(std::ifstream& f, Eigen::MatrixXd& M);
	void save(std::ofstream& f, const Eigen::MatrixXi& M);
	void load(std::ifstream& f, Eigen::MatrixXi& M);
	void save(std::ofstream& f, const Eigen::VectorXd& V);
	void load(std::ifstream& f, Eigen::VectorXd& V);
	void save(std::ofstream& f, const Eigen::VectorXi& V);
	void load(std::ifstream& f, Eigen::VectorXi& V);

	// Forward declarations so the template bodies below can reference each
	// other in any order (two-phase lookup; required by modern GCC).
	template <typename T>
	void save(std::ofstream& f, const std::vector<T>& V);
	template <typename T>
	void load(std::ifstream& f, std::vector<T>& V);
	template <typename T>
	void save(std::ofstream& f, const std::vector<std::vector<T>>& V);
	template <typename T>
	void load(std::ifstream& f, std::vector<std::vector<T>>& V);
	template <typename T1, typename T2>
	void save(std::ofstream& f, const std::vector<std::pair<std::vector<T1>, T2>>& V);
	template <typename T1, typename T2>
	void load(std::ifstream& f, std::vector<std::pair<std::vector<T1>, T2>>& V);

	template <typename T>
	void save(std::ofstream& f, const std::vector<std::vector<T>>& V) {
		f << V.size() << std::endl;
		for (int i = 0; i < V.size(); ++i) {
			save(f, V[i]);
		}
	}

	template <typename T>
	void load(std::ifstream& f, std::vector<std::vector<T>>& V) {
		int s;
		f >> s;
		V.resize(s);
		for (int i = 0; i < s; ++i) {
			load(f, V[i]);
		}
	}

	template <typename T1, typename T2>
	void save(std::ofstream& f, const std::vector<std::pair<std::vector<T1>, T2>>& V) {
		f << V.size() << std::endl;
		for (int i = 0; i < V.size(); ++i) {
			save(f, V[i].first);
			f << V[i].second;
			f << std::endl;
		}
	}

	template <typename T1, typename T2>
	void load(std::ifstream& f, std::vector<std::pair<std::vector<T1>, T2>>& V) {
		int s;
		f >> s;
		V.resize(s);
		for (int i = 0; i < s; ++i) {
			load(f, V[i].first);
			f >> V[i].second;
		}
	}

	template <typename T>
	void save(std::ofstream& f, const std::vector<T>& V) {
		f << V.size() << std::endl;
		for (int i = 0; i < V.size(); ++i) {
			f << V[i];
			if (i < V.size() - 1) {
				f << " ";
			}
		}
		f << std::endl;
	}

	template <typename T>
	void load(std::ifstream& f, std::vector<T>& V) {
		int s;
		T val;
		f >> s;
		V.resize(s);
		for (int i = 0; i < s; ++i) {
			f >> val;
			V[i] = val;
		}
	}
}