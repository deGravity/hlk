#include "disjointset.h"

#include <map>

namespace hlk {
	IntUnionFind::IntUnionFind(int size)
	{
		parent = std::vector<int>(size, -1);
	}
	int IntUnionFind::find(int a)
	{
		while (parent[a] >= 0) {
			if (parent[parent[a]] >= 0) parent[a] = parent[parent[a]];
			a = parent[a];
		}
		return a;
	}

	void IntUnionFind::join(int a, int b)
	{
		a = find(a);
		b = find(b);
		if (a < b) {
			parent[a] += parent[b];
			parent[b] = a;
		}
		else {
			parent[b] += parent[a];
			parent[a] = b;
		}
	}

	std::vector<std::vector<int>> IntUnionFind::components() {
		std::vector<std::vector<int>> components;
		std::map<int, int> rep;
		for (int i = 0; i < parent.size(); ++i) {
			if (parent[i] < 0) {
				rep[i] = components.size();
				components.push_back({i});
			}
		}
		for (int i = 0; i < parent.size(); ++i) {
			if (parent[i] >= 0) {
				components[rep[find(i)]].push_back(i);
			}
		}
		return components;
	}
}