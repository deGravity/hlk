#include<iostream>
#include "scheduler.hpp"

int main_scheduler(int argc, char* argv[]){
	using namespace vkmp;
	std::map<int, std::pair<int,int>> yarn_mappings;
	
	// for plated or fairisle yarn pairs, use both pairs.
	yarn_mappings[0] = std::make_pair(1,-1);
	yarn_mappings[1] = std::make_pair(2,-1);

	//Stitch ascii files have the following format of stitches:
	//yarn-number stitch-type direction in0 in1 out0 out1 at_x at_y at_z stitch-id stitch-name isbasic-type is-custom-xfer
	//stitch-names should map to the stitch names in dsl.js if they are to be used

	// Note: a small amount of yarn handling logic that was
	// removed when extracting this, if using multi yarns,
	// dsl.js should keep track of last used yarn and insert 
	// or eject yarn when starting and ending tube
	{	
		Scheduler s;
		load_stitches("example.st", &s.stitches);
		std::cout << "loaded  " << s.stitches.size() << std::endl;
		s.do_schedule(yarn_mappings, true, -1);
		s.write_schedule("example.out.js");
		//s.dump_svg("example.svg");
	}

	{	
		Scheduler s;
		load_stitches("example2.st", &s.stitches);
		std::cout << "loaded  " << s.stitches.size() << std::endl;
		s.do_schedule(yarn_mappings, true, -1);
		s.write_schedule("example2.out.js");
		//s.dump_svg("example.svg");
	}

	// to process the output files:
	// run  `node example.out.js`
	// produces example.out.k that can be processed with knitout-to-dat.js

	return 0;
}
