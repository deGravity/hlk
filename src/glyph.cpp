#include "glyph.h"

#include <igl/slice_into.h>
#include <igl/slice.h>
#include <igl/cat.h>

namespace hlk {
	void set_glyph(
		const Eigen::MatrixXd& uv_in,
		const Eigen::VectorXi& slot,
		const Eigen::MatrixXd& glyph,
		Eigen::MatrixXd& uv_out)
	{
		// uv_out[slot] = [uv_in[slot] |  1] * glyph
		Eigen::MatrixXd S;
		Eigen::MatrixXd uv_in_slot;
		igl::slice(uv_in, slot, 1, uv_in_slot);
		Eigen::MatrixXd ONE = Eigen::MatrixXd::Ones(slot.size(), 1);
		igl::cat(2, uv_in_slot, ONE, S);
		Eigen::MatrixXd SG = S * glyph;
		igl::slice_into(SG, slot, 1, uv_out);		
	}

	void set_glyph(
		const Eigen::MatrixXd& uv_in,
		const Eigen::VectorXi& slot,
		const Eigen::MatrixXd& glyph,
		const Eigen::RowVector4d& color,
		Eigen::MatrixXd& uv_out,
		Eigen::MatrixXd& color_out)
	{
		set_glyph(uv_in, slot, glyph, uv_out);
		set_color(slot, color, color_out);
	}

	void set_glyph(
		const Eigen::MatrixXd& uv_in,
		const Eigen::VectorXi& slot,
		const Eigen::MatrixXd& glyph,
		const Eigen::RowVector3d& color,
		Eigen::MatrixXd& uv_out,
		Eigen::MatrixXd& color_out)
	{
		set_glyph(uv_in, slot, glyph, uv_out);
		set_color(slot, color, color_out);
	}

	void set_color(
		const Eigen::VectorXi& slot,
		const Eigen::RowVector3d color,
		Eigen::MatrixXd& color_out
	)
	{
		Eigen::RowVector4d alpha_color = Eigen::RowVector4d(color[0], color[1], color[2], 1.0);
		set_color(slot, alpha_color, color_out);
	}

	void set_color(
		const Eigen::VectorXi& slot, 
		const Eigen::RowVector4d color, 
		Eigen::MatrixXd& color_out)
	{
		for (int r = 0; r < slot.size(); ++r) {
			color_out.row(slot[r]) = color;
		}
	}
}