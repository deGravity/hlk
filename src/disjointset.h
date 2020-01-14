#pragma once

#include <vector>

namespace hlk {
	struct IntUnionFind {
		IntUnionFind(int size);
		std::vector<int> parent;
		int find(int a);
		void join(int a, int b);
		std::vector<std::vector<int>> components();
	};
}