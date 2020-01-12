#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>

namespace hlk {

	struct Texture {
		std::string name;
		Eigen::MatrixXd pattern; // 0 = knit, 1 = purl
		Eigen::RowVector4d color;
	};

	static std::vector<Texture> textures;

	// ADD NEW TEXTURES HERE!
	static std::vector<Texture> get_textures() {
		std::vector<Texture> tex;
		Texture stockinette;
		stockinette.name = "Stockinette";
		stockinette.pattern.resize(1, 1);
		stockinette.pattern << 0;
		stockinette.color = Eigen::RowVector4d(1.0, 1.0, 1.0, 1.0);
		tex.push_back(stockinette);

		Texture garter;
		garter.name = "Garter";
		garter.pattern.resize(2, 1);
		garter.pattern << 0, 1;
		garter.color = Eigen::RowVector4d(0.8, 0.8, 0.8, 1.0);
		tex.push_back(garter);

		Texture ribbing;
		ribbing.name = "Ribbing";
		ribbing.pattern.resize(1, 2);
		ribbing.pattern << 0, 1;
		ribbing.color = Eigen::RowVector4d(0.6, 0.6, 0.6, 1.0);
		tex.push_back(ribbing);

		Texture double_moss;
		double_moss.name = "Double Moss";
		double_moss.pattern.resize(4, 4);
		double_moss.pattern <<
			0, 0, 1, 1,
			0, 0, 1, 1,
			1, 1, 0, 0,
			1, 1, 0, 0;
		double_moss.color = Eigen::RowVector4d(0.8, 0.8, 1.0, 1.0);
		tex.push_back(double_moss);

		Texture beaded_rib;
		beaded_rib.name = "Beaded Rib";
		beaded_rib.pattern.resize(2, 6);
		beaded_rib.pattern <<
			1, 0, 1, 0, 1, 0,
			1, 1, 1, 0, 0, 0;
		beaded_rib.color = Eigen::RowVector4d(0.8, 1.0, 0.8, 1.0);
		tex.push_back(beaded_rib);

		return tex;
	}

	static void init_textures() {
		if (textures.size() == 0) {
			textures = get_textures();
		}
	}
}