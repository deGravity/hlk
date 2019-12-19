#ifndef SCHEDULER_H
#define SCHEDULER_H
// Modified autoknit scheduler
#include "xferplan/Stitch.hpp"
#include "xferplan/Shape.hpp"
#include "xferplan/ScheduleCost.hpp"
#include "xferplan/embed_DAG.hpp"
#include "xferplan/plan_transfers.hpp"
#include "xferplan/typeset.hpp"

#include <vector>
#include <map>
#include <queue>
#include <set>


namespace vkmp {

// yarn, canonical direction, list of faces
// typedef	std::vector<std::tuple<int, bool, std::vector<int>>> PATHS;
//Loop held on a needle:
struct Loop {
    constexpr Loop(uint32_t stitch_, uint32_t idx_) : stitch(stitch_), idx(idx_) {
    }
    uint32_t stitch;
    uint32_t idx; // 0 or 1
    bool operator==(Loop const &o) const {
        return (stitch == o.stitch && idx == o.idx);
    }
    bool operator!=(Loop const &o) const {
        return (stitch != o.stitch || idx != o.idx);
    }
    bool operator<(Loop const &o) const {
        if (stitch != o.stitch) return stitch < o.stitch;
        else return idx < o.idx;
    }
    std::string to_string() const {
        if (stitch == -1U && idx == -1U) return "GAP";
        else return std::to_string(stitch) + "_" + std::to_string(idx);
    }
};
constexpr const Loop INVALID_LOOP = Loop(-1U, -1U);


struct CycleIndex {
    CycleIndex(uint32_t cycle_, uint32_t index_) : cycle(cycle_), index(index_) { }
    uint32_t cycle = -1U;
    uint32_t index = -1U;
    bool operator!=(CycleIndex const &o) const {
        return cycle != o.cycle || index != o.index;
    }
    std::string to_string() const {
        if (cycle == -1U && index == -1U) return ".";
        return std::to_string(int32_t(cycle)) + "-" + std::to_string(int32_t(index));
    }
    std::string to_string_simple() const {
        if (cycle == -1U && index == -1U) return ".";
        if (cycle < 26) {
            if (index == 0) {
                return std::string() + char('A' + cycle);
            } else if (index == 1) {
                return std::string() + '+';
            } else {
                return std::string() + char('a' + cycle);
            }
        } else {
            if (index == 0) {
                return std::string() + '*';
            } else {
                return std::string() + 'x';
            }
        }
    }
};

class Scheduler
{
public:
    std::vector<Stitch> stitches;
    std::vector< std::string > instructions;

    Scheduler();
    bool do_schedule( std::map<int, std::pair<int,int>> yarn_mappings, bool cse, int depth);
    bool write_schedule(std::string filename);
    void dump_svg(std::string filename);
};

#endif // SCHEDULER_H

}