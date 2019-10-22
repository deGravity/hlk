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
}