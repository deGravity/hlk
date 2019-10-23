#pragma once
#include<Eigen/Core>

namespace hlk {
namespace glyphs {
static Eigen::MatrixXd SPIRAL = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.9218106995884774, 0.0, 
		0.0, 0.0798004987531172, 
		0.0, 0.9177057356608479;
	return tmp;
}();

static Eigen::MatrixXd SOLID_ARROW = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.9876543209876543, 0.0, 
		0.0, 0.08977556109725686, 
		0.0, 0.8079800498753117;
	return tmp;
}();

static Eigen::MatrixXd THIN_SOLID_LINE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.9053497942386831, 0.0, 
		0.0, 0.08977556109725686, 
		0.0, 0.8079800498753117;
	return tmp;
}();

static Eigen::MatrixXd SOLID_LINE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.9053497942386831, 0.0, 
		0.0, 0.04488778054862843, 
		0.0, 0.830423940149626;
	return tmp;
}();

static Eigen::MatrixXd DASHED_ARROW = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.9876543209876543, 0.0, 
		0.0, 0.09226932668329177, 
		0.1440329218106996, 0.6683291770573566;
	return tmp;
}();

static Eigen::MatrixXd THIN_DASHED_LINE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.8436213991769548, 0.0, 
		0.0, 0.09226932668329177, 
		0.0, 0.6683291770573566;
	return tmp;
}();

static Eigen::MatrixXd DASHED_LINE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.8436213991769548, 0.0, 
		0.0, 0.04488778054862843, 
		0.0, 0.6932668329177057;
	return tmp;
}();

static Eigen::MatrixXd SEAM = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.8888888888888888, 0.0, 
		0.0, 0.04488778054862843, 
		0.00823045267489712, 0.5411471321695761;
	return tmp;
}();

static Eigen::MatrixXd SHORT_ROW = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.01646090534979424, 0.35910224438902744;
	return tmp;
}();

static Eigen::MatrixXd INCREASE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.2674897119341564, 0.3640897755610973;
	return tmp;
}();

static Eigen::MatrixXd LEANING_INCREASE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.5473251028806584, 0.3640897755610973;
	return tmp;
}();

static Eigen::MatrixXd CIRCLE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.7777777777777778, 0.36159600997506236;
	return tmp;
}();

static Eigen::MatrixXd NO_SHORT_ROW = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.0205761316872428, 0.18204488778054864;
	return tmp;
}();

static Eigen::MatrixXd NO_INCREASE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.26337448559670784, 0.17955112219451372;
	return tmp;
}();

static Eigen::MatrixXd NO_LEANING_INCREASE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.51440329218107, 0.18453865336658354;
	return tmp;
}();

static Eigen::MatrixXd NO_CIRCLE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.2139917695473251, 0.0, 
		0.0, 0.12967581047381546, 
		0.7736625514403292, 0.1770573566084788;
	return tmp;
}();

static Eigen::MatrixXd CROSS = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.24279835390946503, 0.0, 
		0.0, 0.14713216957605985, 
		0.012345679012345678, 0.0199501246882793;
	return tmp;
}();

static Eigen::MatrixXd CHECK = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.24279835390946503, 0.0, 
		0.0, 0.14713216957605985, 
		0.26337448559670784, 0.007481296758104738;
	return tmp;
}();

static Eigen::MatrixXd GEAR = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.24279835390946503, 0.0, 
		0.0, 0.14713216957605985, 
		0.5061728395061729, 0.007481296758104738;
	return tmp;
}();

static Eigen::MatrixXd HEART = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.24279835390946503, 0.0, 
		0.0, 0.14713216957605985, 
		0.757201646090535, 0.012468827930174564;
	return tmp;
}();

static Eigen::MatrixXd NONE = [] {
	Eigen::Matrix<double, 3, 2> tmp;
	 tmp <<
		0.09876543209876543, 0.0, 
		0.0, 0.059850374064837904, 
		0.3292181069958848, 0.6109725685785536;
	return tmp;
}();

}
}
