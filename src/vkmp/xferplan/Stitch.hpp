#pragma once

#include <glm/glm.hpp>

#include <vector>
#include <string>
#include <sstream>

namespace vkmp {

// any data for this stitch that comes from the config(lib) file:
// TODO update GLWidget::Stitch, maybe call that StitchConfig actually
// and use the same thing to avoid duplication and keeping things in sync
// and all that:
struct StData{
    uint32_t id = -1U;
    std::string name;
    int is_basic_type = 0; // implies all stitches belonging to this type go into their own pass
    int is_same_bed = 0;   // implies is_basic_type is true, put all these stitches on the same bed
    int is_custom_xfer = 0;// implies don't call transfer planning the code pre-pass will do the needful
    // stitch number is not included because it can be called in the code directly
    bool in_end = false;
};

struct Stitch {
    uint32_t _fid = -1; // extra helper
    //char bed = 'u';
    //int  needle = -1;
    std::pair<char, int> bn_0 = std::make_pair('u',0); // generated loop-0
    std::pair<char, int> bn_1 = std::make_pair('u', 0); // generated loop-1 (if increasing)
    int step = -1; // at what 'step' was this stitch made
	//which yarn the stitch is being made with:
	uint32_t yarn = 0;

    StData data;

	//type of stitch (determines how many of in/out are used):
    // this will be removed: excepte for tracking 's','e','r'
	enum : char {
		//0-in, 1-out:
		Start = 's',
		//1-in, 0-out:
		End = 'e',
		//1-in, 1-out:
		Tuck = 't',
		Miss = 'm',
        Regular = 'k', // I don't want to call this 'r' incase it is interpreted as reverse or something
		//1-in, 2-out:
		Increase = 'i',
		//2-in, 1-out:
		Decrease = 'd',
	};
    struct Cable_Info{
      int cable_id = -1;
      int local_id = -1;
      int stack = -1;
      int connection = -1;
    };
    char type = Regular; // This type is now redundant, but is still used to track what sort of pentagon in make_stitches
    //bool is_cable = false;
    //bool is_knit = true; // else purl
    //bool is_left_leaning = false; //TODO default is what
    //bool is_split = false; // default is yarn-over
    //bool is_decreased = false; // marking stitches that are decreased, not the knit through
    //int  is_decreased_to = -1; // this stitch will be knit through
    Cable_Info cable_info;
    Cable_Info cable_info_secondary; // what happens when you have an increase or decrease ?

	//direction of stitch (relative to current tube):
	enum : char {
		CW = 'c', Clockwise = CW,
		AC = 'a', Anticlockwise = AC, CCW = AC, Counterclockwise = AC,
	};
	char direction = CW;
	//ins and outs are in construction order:
	uint32_t in[2] = {-1U, -1U};
	uint32_t out[2] = {-1U, -1U};
	//DEBUG info:
	glm::vec3 at;

	//helpers:
	uint32_t find_in(uint32_t s) const {
		if (in[0] == s) return 0;
		else if (in[1] == s) return 1;
		else return -1U;
	}
	uint32_t find_out(uint32_t s) const {
		if (out[0] == s) return 0;
		else if (out[1] == s) return 1;
		else return -1U;
	}
	bool check_type() const {

		if (type == Start) {
			return (in[0] == -1U && in[1] == -1U && out[0] != -1U && out[1] == -1U);
		} else if (type == End) {
			return (in[0] != -1U && in[1] == -1U && out[0] == -1U && out[1] == -1U);
        } else if (type == Tuck || type == Miss || type == Regular || type == 'x') {
			return (in[0] != -1U && in[1] == -1U && out[0] != -1U && out[1] == -1U);
		} else if (type == Increase) {
			return (in[0] != -1U && in[1] == -1U && out[0] != -1U && out[1] != -1U);
		} else if (type == Decrease) {
			return (in[0] != -1U && in[1] != -1U && out[0] != -1U && out[1] == -1U);
        }
        else {
			return false;
		}
	}
    bool check_cable() const{
        {
            return (cable_info.cable_id >= 0);
        }
        return true;
    }

	bool check_direction() const {
		return (direction == Clockwise || direction == Anticlockwise);
	}
    std::string to_string(){
        std::ostringstream out;
        out <<"type: " << this->type << " name: " << this->data.name
            << " in " << (int)this->in[0]
            <<", " <<(int)this->in[1] << " out " << (int)this->out[0]
            <<", " <<(int)this->out[1] << " dir " << this->direction
            << " fid " << this->_fid;
        return out.str();
    }

};

bool load_stitches(std::string const &filename, std::vector< Stitch > *into);
void save_stitches(std::string const &filename, std::vector< Stitch > const &from);
void save_stitches_as_CW(std::string const &filename, std::vector< Stitch > const &from);
void save_stitches_as_SM(std::string const &filename, std::vector< Stitch > const &from);

}