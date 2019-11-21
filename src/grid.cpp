#include "grid.h"

#include <list>
#include <numeric>

namespace hlk {

	void compute_row_widths(int base, int top, int width) {

	}

	void Grid::cells_from_diffs(int rows, int cols, std::vector<std::vector<int>> merge_points)
	{
		cells.resize(rows, cols);
		std::list<int> cell_offsets(cols);
		std::iota(cell_offsets.begin(), cell_offsets.end(), 0);

		int curr_cell = 0;
		for (int row = 0; row < rows; ++row) {
			// Populate Current Row based on offsets
			for (int col = 0; col < cols; ++col) {

			}

			// Recompute next row's set of offsets
		}

	}

};