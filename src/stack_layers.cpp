#include "stack_layers.h"

#include <igl/EPS.h>

namespace hlk {
	void offset_layers(
		const Eigen::MatrixXd& V,
		const Eigen::MatrixXd& N,
		const Eigen::VectorXi& layer,
		Eigen::MatrixXd& V_out)
	{
		//V_out = V.array() + N.array().colwise()* layer.array() * igl::DOUBLE_EPS;
	}
}