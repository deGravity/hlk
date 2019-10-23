#pragma once

#include <Eigen/Core>

namespace hlk {
	void set_glyph(
		const Eigen::MatrixXd& uv_in,
		const Eigen::VectorXi& slot,
		const Eigen::MatrixXd& glyph,
		Eigen::MatrixXd& uv_out);

	void set_glyph(
		const Eigen::MatrixXd& uv_in,
		const Eigen::VectorXi& slot,
		const Eigen::MatrixXd& glyph,
		const Eigen::RowVector4d& color,
		Eigen::MatrixXd& uv_out,
		Eigen::MatrixXd& color_out);

	void set_glyph(
		const Eigen::MatrixXd& uv_in,
		const Eigen::VectorXi& slot,
		const Eigen::MatrixXd& glyph,
		const Eigen::RowVector3d& color,
		Eigen::MatrixXd& uv_out,
		Eigen::MatrixXd& color_out);

	void set_color(
		const Eigen::VectorXi& slot,
		const Eigen::RowVector3d color,
		Eigen::MatrixXd& color_out
	);

	void set_color(
		const Eigen::VectorXi& slot,
		const Eigen::RowVector4d color,
		Eigen::MatrixXd& color_out);

	// Color Matrices: TODO - Move these to their own header
	namespace color {
		static Eigen::RowVector4d RED = [] {
			return Eigen::RowVector4d(1.0, 0.0, 0.0, 1.0);
		}();
		static Eigen::RowVector4d GREEN = [] {
			return Eigen::RowVector4d(0.0, 1.0, 0.0, 1.0);
		}();
		static Eigen::RowVector4d BLUE = [] {
			return Eigen::RowVector4d(0.0, 0.0, 1.0, 1.0);
		}();
		static Eigen::RowVector4d WHITE = [] {
			return Eigen::RowVector4d(1.0, 1.0, 1.0, 1.0);
		}();
		static Eigen::RowVector4d BLACK = [] {
			return Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0);
		}();
		static Eigen::RowVector4d INVISIBLE = [] {
			return Eigen::RowVector4d(0.0, 0.0, 0.0, 0.0);
		}();
	}
}