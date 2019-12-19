#ifndef STACKEDPLANNER_H
#define STACKEDPLANNER_H

#include "plan_transfers.hpp" // keep names consistent

namespace vkmp {

	bool stacked_planner(Constraints const& constraints,
		std::vector<BedNeedle> from,
		std::vector<BedNeedle> to,
		std::vector<int> firsts,
		std::vector<Slack> slack, // slack with next
		std::vector<Transfer>* transfers
	);

}

#endif // STACKEDPLANNER_H
