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

		Texture beaded_rib;
		beaded_rib.name = "Beaded Rib";
		beaded_rib.pattern.resize(2, 6);
		beaded_rib.pattern <<
			1, 0, 1, 0, 1, 0,
			1, 1, 1, 0, 0, 0;
		beaded_rib.color = Eigen::RowVector4d(0.8, 1.0, 0.8, 1.0);
		tex.push_back(beaded_rib);

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

        Texture moss_parallel;
        moss_parallel.name = "Moss Parallelograms";
        moss_parallel.pattern.resize(6, 10);
        moss_parallel.pattern <<
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 1, 0, 1, 0,
            0, 0, 0, 1, 0, 1, 0, 1, 0, 0,
            0, 0, 1, 0, 1, 0, 1, 0, 0, 0,
            0, 1, 0, 1, 0, 1, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 0, 0, 0, 0;
        moss_parallel.color = Eigen::RowVector4d(0.4, 0.4, 0.8, 1.0);
        tex.push_back(moss_parallel);

        Texture moss_zigzag;
        moss_zigzag.name = "Moss Zigzag";
        moss_zigzag.pattern.resize(10, 9);
        moss_zigzag.pattern <<
            0, 0, 0, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 1, 0, 1, 0, 1, 0,
            0, 0, 1, 0, 1, 0, 1, 0, 0,
            0, 1, 0, 1, 0, 1, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 0, 0, 0,
            0, 1, 0, 1, 0, 1, 0, 0, 0,
            0, 0, 1, 0, 1, 0, 1, 0, 0,
            0, 0, 0, 1, 0, 1, 0, 1, 0,
            0, 0, 0, 0, 1, 0, 1, 0, 1,
            1, 0, 0, 0, 0, 1, 0, 1, 0;
        moss_zigzag.color = Eigen::RowVector4d(0.4, 0.8, 0.4, 1.0);
        tex.push_back(moss_zigzag);

        Texture waffled_squares;
        waffled_squares.name = "Waffled Squares";
        waffled_squares.pattern.resize(4, 3);
        waffled_squares.pattern <<
            1, 1, 1,
            1, 0, 0,
            1, 0, 0,
            1, 0, 0;
        waffled_squares.color = Eigen::RowVector4d(0.6, 0.8, 1.0, 1.0);
        tex.push_back(waffled_squares);

        Texture pennants;
        pennants.name = "Pennants";
        pennants.pattern.resize(6, 5);
        pennants.pattern <<
            1, 1, 1, 1, 1,
            1, 1, 1, 1, 0,
            1, 1, 1, 0, 0,
            1, 1, 0, 0, 0,
            1, 0, 0, 0, 0,
            0, 0, 0, 0, 0;
        pennants.color = Eigen::RowVector4d(0.9, 0.2, 0.5, 1.0);
        tex.push_back(pennants);

        Texture lizard_lattice;
        lizard_lattice.name = "Lizard Lattice";
        lizard_lattice.pattern.resize(18, 6);
        lizard_lattice.pattern <<
            0, 0, 0, 1, 1, 1,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 1, 1,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 1, 1,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            1, 1, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            1, 1, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            1, 1, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;
        lizard_lattice.color = Eigen::RowVector4d(0.8, 0.4, 0.4, 1.0);
        tex.push_back(lizard_lattice);

        Texture textured_stripe;
        textured_stripe.name = "Textured Stripe";
        textured_stripe.pattern.resize(12, 3);
        textured_stripe.pattern <<
            0, 1, 1,
            0, 1, 1,
            0, 1, 1,
            0, 1, 1,
            0, 1, 0,
            0, 1, 0,
            0, 1, 0,
            0, 1, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;
        textured_stripe.color = Eigen::RowVector4d(0.2, 0.9, 0.5, 1.0);
        tex.push_back(textured_stripe);

        Texture diamond_brocade;
        diamond_brocade.name = "Diamond Brocade";
        diamond_brocade.pattern.resize(8, 8);
        diamond_brocade.pattern <<
            0, 0, 1, 0, 1, 0, 0, 0,
            0, 1, 0, 0, 0, 1, 0, 0,
            1, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1,
            1, 0, 0, 0, 0, 0, 1, 0,
            0, 1, 0, 0, 0, 1, 0, 0,
            0, 0, 1, 0, 1, 0, 0, 0;
        diamond_brocade.color = Eigen::RowVector4d(0.6, 1.0, 0.8, 1.0);
        tex.push_back(diamond_brocade);

        Texture ridge_and_diamond_stripes;
        ridge_and_diamond_stripes.name = "Ridge & Diamond Stripes";
        ridge_and_diamond_stripes.pattern.resize(12, 8);
        ridge_and_diamond_stripes.pattern <<
            0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 1, 1, 0, 1, 1, 1,
            0, 1, 1, 0, 0, 0, 1, 1,
            1, 1, 0, 0, 0, 0, 0, 1,
            1, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 0, 0, 0, 0, 0, 1,
            0, 1, 1, 0, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 1, 1, 1,
            0, 1, 1, 1, 1, 1, 1, 1;
        ridge_and_diamond_stripes.color = Eigen::RowVector4d(0.9, 0.5, 0.2, 1.0);
        tex.push_back(ridge_and_diamond_stripes);

        Texture right_diagonal_brocade;
        right_diagonal_brocade.name = "Right Diagonal Brocade";
        right_diagonal_brocade.pattern.resize(10, 5);
        right_diagonal_brocade.pattern <<
            0, 0, 0, 0, 1,
            0, 0, 0, 0, 1,
            0, 0, 0, 1, 0,
            0, 0, 0, 1, 0,
            0, 0, 1, 0, 0,
            0, 0, 1, 0, 0,
            0, 1, 0, 0, 0,
            0, 1, 0, 0, 0,
            1, 0, 0, 0, 0,
            1, 0, 0, 0, 0;
        right_diagonal_brocade.color = Eigen::RowVector4d(1.0, 0.8, 0.6, 1.0);
        tex.push_back(right_diagonal_brocade);

        Texture left_diagonal_brocade;
        left_diagonal_brocade.name = "Left Diagonal Brocade";
        left_diagonal_brocade.pattern.resize(10, 5);
        left_diagonal_brocade.pattern <<
            1, 0, 0, 0, 0,
            1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1,
            0, 0, 0, 0, 1;
        left_diagonal_brocade.color = Eigen::RowVector4d(1.0, 0.6, 0.8, 1.0);
        tex.push_back(left_diagonal_brocade);

        Texture stacked_triangle;
        stacked_triangle.name = "Stacked Triangle";
        stacked_triangle.pattern.resize(12, 12);
        stacked_triangle.pattern <<
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
            0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
            0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
            0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1,
            1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
        stacked_triangle.color = Eigen::RowVector4d(0.8, 0.6, 1.0, 1.0);
        tex.push_back(stacked_triangle);

        Texture stacked_blocks;
        stacked_blocks.name = "Stacked Blocks/Hexagon";
        stacked_blocks.pattern.resize(20, 10);
        stacked_blocks.pattern <<
            0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
            1, 0, 1, 0, 0, 1, 1, 0, 1, 0,
            0, 1, 0, 0, 0, 1, 1, 1, 0, 1,
            1, 0, 0, 0, 0, 1, 1, 1, 1, 0,
            0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 1, 0, 1, 1, 1, 1,
            0, 0, 0, 1, 0, 1, 0, 1, 1, 1,
            0, 0, 1, 0, 1, 0, 1, 0, 1, 1,
            0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
            1, 1, 0, 1, 0, 1, 0, 1, 0, 0,
            1, 1, 1, 0, 1, 0, 1, 0, 0, 0,
            1, 1, 1, 1, 0, 1, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 0, 0, 0, 0, 1,
            1, 0, 1, 1, 1, 0, 0, 0, 1, 0,
            0, 1, 0, 1, 1, 0, 0, 1, 0, 1,
            1, 0, 1, 0, 1, 0, 1, 0, 1, 0;
        stacked_blocks.color = Eigen::RowVector4d(0.8, 1.0, 0.6, 1.0);
        tex.push_back(stacked_blocks);

        Texture basketweave;
        basketweave.name = "5x5 Basketweave";
        basketweave.pattern.resize(10, 10);
        basketweave.pattern <<
            0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 0, 0, 0, 0;
        basketweave.color = Eigen::RowVector4d(0.2, 0.5, 0.9, 1.0);
        tex.push_back(basketweave);

        Texture expanded_basketweave;
        expanded_basketweave.name = "Expanded Basketweave";
        expanded_basketweave.pattern.resize(22, 14);
        expanded_basketweave.pattern <<
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0;
        expanded_basketweave.color = Eigen::RowVector4d(0.7, 0.3, 0.1, 1.0);
        tex.push_back(expanded_basketweave);

        Texture goblets;
        goblets.name = "Goblets";
        goblets.pattern.resize(10, 6);
        goblets.pattern <<
            1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1,
            1, 0, 0, 0, 0, 1,
            1, 0, 0, 0, 0, 1,
            1, 0, 0, 0, 0, 1,
            1, 0, 0, 0, 0, 1,
            1, 1, 0, 0, 1, 1,
            1, 1, 0, 0, 1, 1,
            1, 1, 0, 0, 1, 1,
            1, 1, 0, 0, 1, 1;
        goblets.color = Eigen::RowVector4d(0.5, 0.2, 0.9, 1.0);
        tex.push_back(goblets);

        Texture squared_checks;
        squared_checks.name = "Squared Checks";
        squared_checks.pattern.resize(12, 10);
        squared_checks.pattern <<
            0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 1, 1, 0, 0, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 0, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 0, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 0, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 0, 0, 0, 1, 1,
            0, 0, 1, 1, 0, 0, 0, 0, 1, 1,
            0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        squared_checks.color = Eigen::RowVector4d(0.5, 0.9, 0.2, 1.0);
        tex.push_back(squared_checks);

        Texture fence_posts;
        fence_posts.name = "Fence Posts";
        fence_posts.pattern.resize(6, 11);
        fence_posts.pattern <<
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1,
            1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1;
        fence_posts.color = Eigen::RowVector4d(0.1, 0.3, 0.7, 1.0);
        tex.push_back(fence_posts);

        Texture art_deco;
        art_deco.name = "Pinwheel";
        art_deco.pattern.resize(20, 12);
        art_deco.pattern <<
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
        art_deco.color = Eigen::RowVector4d(0.3, 0.7, 0.1, 1.0);
        tex.push_back(art_deco);

		return tex;
	}

	static void init_textures() {
		if (textures.size() == 0) {
			textures = get_textures();
		}
	}
}