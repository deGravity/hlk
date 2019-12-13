#pragma once

#include <glm/glm.hpp>
#include <Eigen/Core>

#include <vector>
#include <string>
#include <algorithm>

// The autoknit pipeline in data formats and transformation functions.

namespace ak {


	struct Stitch {
		//which yarn the stitch is being made with:
		uint32_t yarn = 0;
		//type of stitch (determines how many of in/out are used):
		enum : char {
			//0-in, 1-out:
			Start = 's',
			//1-in, 0-out:
			End = 'e',
			//1-in, 1-out:
			Tuck = 't',
			Miss = 'm',
			Knit = 'k',
			//TODO: Purl = 'p',
			//1-in, 2-out:
			Increase = 'i',
			//2-in, 1-out:
			Decrease = 'd',
		};
		char type = Knit;
		//direction of stitch (relative to current tube):
		enum : char {
			CW = 'c', Clockwise = CW,
			AC = 'a', Anticlockwise = AC, CCW = AC, Counterclockwise = AC,
		};
		char direction = CW;
		//ins and outs are in construction order:
		uint32_t in[2] = { -1U, -1U };
		uint32_t out[2] = { -1U, -1U };
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
			}
			else if (type == End) {
				return (in[0] != -1U && in[1] == -1U && out[0] == -1U && out[1] == -1U);
			}
			else if (type == Tuck || type == Miss || type == Knit) {
				return (in[0] != -1U && in[1] == -1U && out[0] != -1U && out[1] == -1U);
			}
			else if (type == Increase) {
				return (in[0] != -1U && in[1] == -1U && out[0] != -1U && out[1] != -1U);
			}
			else if (type == Decrease) {
				return (in[0] != -1U && in[1] != -1U && out[0] != -1U && out[1] == -1U);
			}
			else {
				return false;
			}
		}
		bool check_direction() const {
			return (direction == Clockwise || direction == Anticlockwise);
		}

	};

	bool load_stitches(std::string const& filename, std::vector< Stitch >* into);
	void save_stitches(std::string const& filename, std::vector< Stitch > const& from);

// Parameters: used to influence various steps
struct Parameters {
	//stitch size in millimeters:
	float stitch_width_mm = 3.66f;
	float stitch_height_mm = 1.73f;

	//model unit size in millimeters:
	float model_units_mm = 1.0f;

	//maximum edge length for embed_constraints:
	float get_max_edge_length() const {
		return 0.5f * std::min(stitch_width_mm, 2.0f * stitch_height_mm) / model_units_mm;
	}

	//sample spacing for sample_chain:
	float get_chain_sample_spacing() const {
		return 0.25f * stitch_width_mm / model_units_mm;
	}

	//edge sample spacing for embedded_path:
	float get_max_path_sample_spacing() const {
		return 0.02f * std::min(stitch_width_mm, 2.0f * stitch_height_mm) / model_units_mm;
	}
};


struct RowColGraph {
	struct Vertex {
		Eigen::RowVector3d at;
		uint32_t row_in = -1U;
		uint32_t row_out = -1U;
		uint32_t col_in[2] = {-1U, -1U};
		uint32_t col_out[2] = {-1U, -1U};
		void add_col_in(uint32_t i) {
			if (col_in[0] == -1U) col_in[0] = i;
			else if (col_in[1] == -1U) col_in[1] = i;
			else assert(col_in[0] == -1U || col_in[1] == -1U); //no room!
		}
		void add_col_out(uint32_t i) {
			if (col_out[0] == -1U) col_out[0] = i;
			else if (col_out[1] == -1U) col_out[1] = i;
			else assert(col_out[0] == -1U || col_out[1] == -1U); //no room!
		}
	};
	std::vector< Vertex > vertices;
	void clear() {
		vertices.clear();
	}
};

struct TracedStitch {
	uint32_t yarn = -1U; //yarn ID (why is this on a yarn_in? I guess the schedule.cpp code will tell me someday.
	//ins and outs are in construction order (OLD was: CW direction):
	uint32_t ins[2] = {-1U, -1U};
	uint32_t outs[2] = {-1U, -1U};
	enum Type : char {
		None = '\0',
		Start = 's',
		End = 'e',
		Tuck = 't',
		Miss = 'm',
		Knit = 'k',
		//I am going to add these because the scheduling re-write cares about them, though I'm not sure if they are a good idea to have in a general sense:
		Increase = 'i',
		Decrease = 'd',
	} type = None;
	enum Dir : char {
		CW = 'c', Clockwise = CW,
		AC = 'a', Anticlockwise = AC, CCW = AC, Counterclocwise = AC,
	} dir = AC;

	//useful for debugging and visualization:
	uint32_t vertex = -1U; //vertex of rowcolgraph where created
	glm::vec3 at;
};

//cycles -> stitches

void trace_graph(
	RowColGraph const &graph, //in: row-column graph
	std::vector< TracedStitch > *traced //out:traced list of stitches
);

void save_traced(const std::string& save_traced_file, const std::vector<TracedStitch>& traced);


} //namespace ak
