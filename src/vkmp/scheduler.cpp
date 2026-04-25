#include <cstdint>
#include "scheduler.hpp"
#include <deque>
#include <map>
#include <set>
#include <algorithm>
#include <list>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include "xferplan/stackedplanner.h"

#include <Eigen/Eigen>

namespace vkmp {

typedef Eigen::Vector3d Vector3;
#define STITCH_ID_YARNEND 0 // extracted scheduler: no other ids are explicitly used here, so defining yarnend here 
Scheduler::Scheduler()
{

}
//	std::vector<std::tuple<int, bool, std::vector<in0t>>> mKnittingPath;
#if 0
bool Scheduler::make_stitches(Polyhedron* P){
    if(P->mKnittingPath.empty()){
        std::cout <<"No traced faces. early out" << std::endl;
        return false;
    }

    std::set<int> visited_face;
    for(auto p : P->mKnittingPath){
        auto path = std::get<2>(p);
        for(auto fi : path){
            assert(visited_face.count(fi) == 0);
            visited_face.insert(fi);
        }
    }

    stitches.clear();
    //TODO also clear whatever was assigned to faces
    std::cout <<"Pattern has " << P->mCables.size() << " cables. " << std::endl;

    std::map<int, std::pair<std::pair<int,int>, std::pair<int,int>> > edge_to_id_connectionStack;
    if(P->mCables.size()){
        // associate cables with faces
        for(auto  &cables : P->mCables){
            int cid = &cables - &P->mCables[0];
            int n = cables.size() ;
            int idx = 0;
            for(auto x : cables){
                int eid = std::get<0>(x);
                int conn = std::get<1>(x);
                int stack = std::get<2>(x);
                auto he = P->halfedge(eid);
                assert(he->TestFlag(EFLAG_COURSE));
                if(he->TestFlag(EFLAG_BTM)){
                    he = he->twin();
                }
                assert(he->TestFlag(EFLAG_TOP));
                assert(!edge_to_id_connectionStack.count(he->index()));
                edge_to_id_connectionStack[he->index()] = std::make_pair(std::make_pair(cid,idx), std::make_pair(conn, stack));
                idx++;
                std::cout << "\t[cable]Added  " << he->index() << " to map. " << std::endl;
            }
        }
    }
    std::cout << "cable map entries = " << edge_to_id_connectionStack.size() << std::endl;
    std::vector<bool> processed_faces;
    processed_faces.assign(P->numFaces(), false);
    int row_count = 0;

    std::cout << "Num knitting paths = " << P->mKnittingPath.size() << std::endl;
    std::map<int,int> face_stitch_map, stitch_face_map;
    int extra = P->numFaces();

    for(auto &path : P->mKnittingPath){
        int path_id = &path - &P->mKnittingPath[0];
        std::cout << "Path: " << &path - &P->mKnittingPath[0] << std::endl;
        int yarn_num =  std::get<0>(path)+1;// until multi yarn is setup, std::get<0>(path); // using yarn "1" for umbilical
        //assert(P->yarn_mappings.count(yarn_num));
        //yarn_num = P->yarn_mappings[yarn_num].first;
        // this is inverted with my convention?
        bool dir = std::get<1>(path);
        std::vector<int> knittingFaces = std::get<2>(path);
        assert( !knittingFaces.empty() );
        for(auto &sid : knittingFaces){
            auto f = P->face(sid);
            if(!f) continue; // why ?
            assert(!f->hole());
            assert(f->GetBtmCount()+f->GetTopCount() < 4); // no hexagons
            if(f->TestFlag(FFLAG_SHORTROW) && (sid  != knittingFaces[0])){
               dir = !dir;
            }
            if(f->TestFlag(FFLAG_SHORTROW) && (sid  == knittingFaces[0])){
               std::cout <<"First stitch is a short-row " << std::endl;
            }

            {
                Stitch s;

                s._fid = sid;
                s.yarn = yarn_num;
                s.direction = (dir ? Stitch::AC : Stitch::CW); //anticlockwise
                s.type = Stitch::Regular;
                //s.is_knit = f->IsKnit();


                std::vector<GLWidget::Stitch>& stLib = TheGLWidget()->GetStitchLib();
                std::vector<int>& stMap = TheGLWidget()->GetStitchLibMap();
                auto const &config = stLib[stMap[f->GetStitchTypeID()]];
                s.data.id = f->GetStitchTypeID();
                s.data.name = config.mName;
                s.data.is_basic_type = config.is_basic_type;
                s.data.is_custom_xfer = config.ignores_xfer;
                s.data.is_same_bed    = config.is_same_bed;

                if(s.data.id == STITCH_ID_YARNEND && sid == knittingFaces[0]){
                    s.data.in_end =  true;
                }
                assert(f->GetBtmCount()+f->GetTopCount() < 4); // no hexagons
                if(f->GetBtmCount() > 1){
                    s.type = Stitch::Decrease;
                }
                else if(f->GetTopCount() > 1){
                    s.type = Stitch::Increase;
                }
                if(f->TestFlag(FFLAG_SHORTROW)){
                    if(sid  != knittingFaces[0])
                        s.type = 'x';
                    else
                        s.type = Stitch::Tuck;
                }



                s.in[0] = f->GetBotFace_S();
                s.out[0] = f->GetTopFace_S();
                s.in[1] = f->GetBotFaceDec_S();
                s.out[1] = f->GetTopFaceInc_S();

                assert(s.in[0] == -1 || processed_faces[s.in[0]]);
                assert(s.in[1] == -1 || processed_faces[s.in[1]]);
                assert(s.out[0] == -1 || !processed_faces[s.out[0]]);
                assert(s.out[1] == -1 || !processed_faces[s.out[1]]);

                int t0 = f->GetTopEdge_S();
                int t1 = f->GetTopEdgeInc_S();

                assert(t0 != -1);
                assert(P->halfedge(t0)->TestFlag(EFLAG_COURSE));
                assert(P->halfedge(t0)->TestFlag(EFLAG_TOP));

                if(t0 != -1 && edge_to_id_connectionStack.count(t0)){
                    std::cout << "found cable in map " << t0 << std::endl;
                    auto ci = edge_to_id_connectionStack[t0];
                    //s.is_cable = true;
                    // this is all local information and not affected by any stitch/face index mapping:
                    s.cable_info.cable_id = ci.first.first;
                    s.cable_info.local_id = ci.first.second;
                    s.cable_info.connection = ci.second.first;
                    s.cable_info.stack = ci.second.second;
                }

                // this is going to be slightly awkward to deal with when the first stitch is not a part of the cable because the yarn
                // is moving the other way..
                // TODO
                /*
                if(t1 != -1 && edge_to_id_connectionStack.count(t1)){
                    auto ci = edge_to_id_connectionStack[t1];
                    s.is_cable = true;
                    s.cable_info_secondary.cable_id = ci.first;
                    s.cable_info_secondary.connection = ci.second.first;
                    s.cable_info_secondary.stack = ci.second.second;
                }
                */
                if(s.type == Stitch::Decrease){
                    // must have in1 unless in the bottom-most row
                    assert(s.in[1] != -1 || s.in[0] == -1);
                    assert(f->GetBtmCount() == 2);
                    if(dir)
                    std::swap(s.in[0], s.in[1]);

                    if(s.in[0] != -1){
                        // mark the stitches participating in this decrease
                        //stitches[s.in[0]].is_decreased = true;
                        //stitches[s.in[0]].is_decreased_to = stitches.size();
                        assert(s.in[1] != -1);
                        //stitches[s.in[1]].is_decreased = true;
                        //stitches[s.in[1]].is_decreased_to = stitches.size();
                    }
                }
                if(s.type == Stitch::Increase){
                    // must have out1, unless in the topmost row
                    assert(s.out[1] != -1
                            || s.out[0] == -1);
                    assert(f->GetTopCount() == 2);
                    if(dir)
                    std::swap(s.out[0], s.out[1]);

                }
                s.at.x = f->centroid().x();
                s.at.y = f->centroid().y();
                s.at.z = f->centroid().z();

                processed_faces[sid] = true;
                face_stitch_map[sid] = stitches.size();
                stitch_face_map[stitches.size()] = sid;
                stitches.push_back(s);
                //std::cout <<"Processed face " << sid << std::endl;
            }
        } // path faces


 }



    face_stitch_map[-1] = -1;
    stitch_face_map[-1] = -1;
    // make sure stitch ids are consistent:
    for(auto &s : stitches){
        for(int i = 0; i < 2; i++){
            s.in[i]  = face_stitch_map[s.in[i]];
            s.out[i] = face_stitch_map[s.out[i]];

        }
    }



    // duplicate short-row corners:
    // TODO: Check if this can be avoided altogether by just storing short-row that gets translated into a tuck, reverse, miss
    std::vector<GLWidget::Stitch>& stLib = TheGLWidget()->GetStitchLib();
    std::vector<int>& stMap = TheGLWidget()->GetStitchLibMap();
    bool updated = true;
    while(updated){
       // std::cout <<"breaking short row corners" << std::endl;
        updated = false;
        for(int i = 0; i < stitches.size(); i++){
            auto &s = stitches[i];
            if(s.type == 'x'){
                bool do_tuck = (s.data.id == STITCH_ID_SHORTROW);
                //std::cout << s.to_string() << std::endl;
                Stitch ss;
                ss.direction = s.direction;
                s.direction = (s.direction == Stitch::AC ? Stitch::CW : Stitch::AC);

                auto const &knit_config = stLib[stMap[STITCH_ID_KNIT]];
                auto const &tuck_config = stLib[stMap[STITCH_ID_TUCK]];
                auto const &miss_config = stLib[stMap[STITCH_ID_MISS]];
                if(do_tuck){
                s.data.id = tuck_config.mID;
                s.data.name = tuck_config.mName;
                s.data.is_basic_type = tuck_config.is_basic_type;
                s.data.is_custom_xfer =tuck_config.ignores_xfer;
                s.data.is_same_bed    =tuck_config.is_same_bed;

                ss.data.id = miss_config.mID;
                ss.data.name = miss_config.mName;
                ss.data.is_basic_type = miss_config.is_basic_type;
                ss.data.is_custom_xfer =miss_config.ignores_xfer;
                ss.data.is_same_bed    =miss_config.is_same_bed;

                s.type = Stitch::Tuck;
                ss.type = Stitch::Miss;
                }
                else{
                    s.data.id = miss_config.mID;
                    s.data.name = miss_config.mName;
                    s.data.is_basic_type = miss_config.is_basic_type;
                    s.data.is_custom_xfer =miss_config.ignores_xfer;
                    s.data.is_same_bed    =miss_config.is_same_bed;

                    ss.data.id = miss_config.mID;
                    ss.data.name = miss_config.mName;
                    ss.data.is_basic_type = miss_config.is_basic_type;
                    ss.data.is_custom_xfer =miss_config.ignores_xfer;
                    ss.data.is_same_bed    =miss_config.is_same_bed;
                    s.type = Stitch::Regular;
                    ss.type = Stitch::Regular;

                }
                ss.yarn = s.yarn;
                ss._fid = s._fid;
                auto n = P->face(s._fid)->normal();
                ss.at = s.at +  P->mAvgWaleEdgeLength* glm::vec3(n.x(),n.y(),n.z());

                //  s is the tuck stitch
                // ss is the miss stitch
                // ss has to appear after s in the stitch list
                // out for s has been fixed, in for ss has been fixed
                // any index that is greater than i has to be updated by 1
                int t = s.out[0];

                for(auto &p : stitches){
                    for(int k = 0; k < 2; k++){
                        if(p.in[k] != -1 && p.in[k] > i){
                            p.in[k] += 1;
                        }
                        if(p.out[k] != -1 && p.out[k] > i){
                            p.out[k] += 1;
                        }
                    }
                }
                ss.out[0] = s.out[0];
                ss.in[0] = i;
                s.out[0] = i+1;
                s.out[1] = -1;
                s.in[1] = -1;
               // std::cout <<"t = " << t << " t.in = " << stitches[t].in[0] << " i = " << i << std::endl;
                assert(stitches[t].in[0] == i || stitches[t].in[1] == i);
                if(stitches[t].in[0] == i){
                    stitches[t].in[0] = i+1;
                }
                else{
                    stitches[t].in[1] = i+1;
                }

                stitches.insert(stitches.begin()+i+1, ss);
                updated = true;
                break;
            }
        }
    }


    // deal with umbilicals:
    // breaking faces, so dealing with umbilicals the simple way
    for(auto &st : stitches){
        if(st.in[0] == -1){
            assert(st.in[1] == -1);
            if(st.data.id == STITCH_ID_YARNEND){
                stitches[st.out[0]].data.id = STITCH_ID_YARNEND;
                stitches[st.out[0]].data.in_end = true;
            }
            st.type = Stitch::Start;
            st.yarn = 0;// umbilicals use yarn 0

        }
        if(st.out[0] == -1){
            assert(st.out[1] == -1);
            if(st.data.id == STITCH_ID_YARNEND){
                stitches[st.in[0]].data.id = STITCH_ID_YARNEND;
            }
            st.type = Stitch::End;
            st.yarn = 0; //umbilicals use yarn 0
        }
    }

    if(0){
    int start_stitches = 0;
    int end_stitches = 0;
    for(auto &st : stitches){
        if(st.in[0] == -1){
            assert(st._fid != -1);
            start_stitches += P->face(st._fid)->GetBtmCount();
        }
        if(st.out[0] == -1){
            //assert(st.type != Stitch::Start);
            //st.type = Stitch::End;
            assert(st._fid != -1);
            end_stitches += P->face(st._fid)->GetTopCount();
        }


    }
    std::cout <<"Dealing with starts and ends." << std::endl;
    for(auto &st: stitches){
        for(int i = 0; i < 2; i++){
        if(st.in[i] != -1) st.in[i] += start_stitches;
        if(st.out[i] != -1) st.out[i] += start_stitches;
        }
        //if(st.is_decreased_to != -1) st.is_decreased_to += start_stitches;
    }
    int s_i = 0;
    int e_i = stitches.size()+start_stitches;
    std::vector<Stitch> start_list;
    std::vector<Stitch> end_list;
    for(auto &st : stitches){
        int id = &st - &stitches[0];
        if(st.in[0] == -1){
            Stitch s = st;
            s.type = Stitch::Start;
            s._fid = -1;
            //s.is_cable = false;
            s.in[0] = s.in[1] = -1;
            s.out[0] = start_stitches + id;
            s.out[1] = -1;
            s.yarn = 1;
            start_list.push_back(s);
            st.in[0] = s_i;
            s_i++;

            if(st.type == Stitch::Decrease){
                assert(P->face(st._fid)->GetBtmCount() > 1);
            }
            if(P->face(st._fid)->GetBtmCount() > 1){
                assert(st.type == Stitch::Decrease);
                Stitch s = st;
                s.yarn = 1;
                s.type = Stitch::Start;
                s._fid = -1;
                //s.is_cable = false;
                s.in[0] = s.in[1] = -1;
                s.out[0] = start_stitches +id;
                s.out[1] = -1;
                start_list.push_back(s);
                st.in[1] = s_i;
                s_i++;
            }

        }
        if(st.out[0] == -1){
            Stitch s = st;
            s.type = Stitch::End;
            s._fid = -1;
            //s.is_cable = false;
            s.in[0] = id + start_stitches;
            s.in[1] = -1;
            s.out[0] = s.out[1] = -1;
            s.yarn = 1;
            end_list.push_back(s);
            st.out[0] = e_i++;

            if(P->face(st._fid)->GetTopCount() > 1){
                assert(st.type == Stitch::Increase);
                Stitch s = st;
                s.yarn = 1;
                s.type = Stitch::End;
                s._fid = -1;
                //s.is_cable = false;
                s.in[0] = id + start_stitches;
                s.in[1] = -1;
                s.out[0] = s.out[1] = -1;
                end_list.push_back(s);
                st.out[1] = e_i++;

            }
        }
    }
    assert(s_i == start_stitches);
    stitches.insert(stitches.begin(),start_list.begin(),start_list.end());
    stitches.insert(stitches.end(), end_list.begin(), end_list.end());
    assert(e_i == stitches.size());
    }

    std::cout <<"Made #"<<stitches.size() <<" stitches."<<std::endl;
    std::cout <<"Rows #"<<row_count << std::endl;
    int unprocessed = 0;
    int no_face_assigned = 0;
    for(auto fit = P->fBegin(); fit != P->fEnd(); fit++){

        auto f = *fit;
        if(!f) continue; // what?
        if(!f->hole() && !processed_faces[f->index()]){
            unprocessed++;
        }

    }
    for(auto s : stitches){
        if(s._fid < 0 && !(s.type == Stitch::Start || s.type == Stitch::End)){
            no_face_assigned++;
        }
        else if(s._fid >=0 && s._fid < processed_faces.size()){
        processed_faces[s._fid] = false; // unset
        }
    }
    assert(no_face_assigned == 0 && "stitch was assigned no face?");
    for(int i = 0; i < processed_faces.size(); i++){
        if(processed_faces[i]){
            std::cout << "Face " << i << " was assigned to no stitch ?" << std::endl;
            assert(false);
        }
    }
    std::cout <<"#unprocessed faces = " << unprocessed << std::endl;
    assert(unprocessed == 0);

    // debug files:
    // save_stitches("/Users/nvidya/Textiles-Lab/autoknit/debug.st", stitches);
     save_stitches_as_CW("/Users/nvidya/Textiles-Lab/debug.cw", stitches);


     // sanity check the id types for the stitches that we have
     // so the front-end does not keep increases and decreases as
     // particular stitch types ?
     // What about cables ?
     /*
     for(auto &st : stitches){
         assert(st.stitch_type_id == STITCH_ID_KNIT ||
                st.stitch_type_id == STITCH_ID_PURL ||
                st.stitch_type_id == STITCH_ID_MISS ||
                st.stitch_type_id == STITCH_ID_TUCK ||
                st.stitch_type_id == STITCH_ID_YARNEND);
     }*/


    return true;
}

#endif


// new schedule, should potentially be simpler since this only calls the function on whatever input
// but needs to know block in and out size...
bool Scheduler::do_schedule( std::map<int, std::pair<int,int>> yarn_mappings, bool cse, int depth){

    std::cout <<"Begin scheduling. "<<std::endl;
    //New scheduling workflow:
    // (1) split into "steps" ==> bits of knitting that will be done at once
    //    - each step eventually needs a shape + roll + offset for its output loops
    //      (implies a shape for input loops)
    // (2) pick a consistent shape + roll for "interesting" steps
    //    - these are steps that take loops from more than one output or input
    //    - effectively, this finds an upward planar embedding, where the edges are chains of construction steps and the vertices occur when steps have more than one tube as a parent or child.
    // (2) figure out a layout (shape + roll) for each step


    //Construction will proceed in "Steps".
    // Each step will:
    //  a) shift loops stored on the bed, if needed.
    //  b) split/merge cycles if needed.
    //  c) reshape stored loops (roll/increase/decrease)
    //  d) create some new stitches

    // In other words, we can build a graph where the edges are loops stored on the bed, and the vertices are steps.
    // Scheduling means assigning compatible shapes to each edge (equiv: each shape's output)

    std::set<int> active_carriers; // carriers that are currently active on the machine
    //Storage connects steps:
    typedef uint32_t StorageIdx;
    struct Storage : std::deque< Loop > {
        //Shape shape;
    };

    struct ScheduleOption {
        ScheduleCost cost;
        std::vector< uint32_t > in_order; //indexes into step.in
        std::vector< PackedShape > in_shapes; //in_shapes[i] is shape for step.in[i]
        PackedShape inter_shape;
        std::vector< uint32_t > out_order; //indexes into step.out
        std::vector< PackedShape > out_shapes; //out_shapes[i] is shape for step.out[i]
    };

    struct Step {
        // end is not inclusive
        uint32_t begin = 0, end = 0; //stitch range constructed in this step
        std::vector< StorageIdx > in, out;
        Storage inter; //configuration of loops after input arrangement but before performing stitches
        std::vector< uint32_t > inter_to_out; //how stitches in inter are transformed to get to out

        //Step reinterprets storages[in[*]] as inter + storages[out[1...]]
        // stretches and rolls inter,
        // and performs knitting on inter to produce storages[out[0]]

        std::vector< ScheduleOption > options;

    };

    #define REPORT_ERROR( X ) do { std::cerr << (X) << std::endl; exit(1); } while(0)

    std::vector< Storage > storages;
    std::vector< Step > steps;

    storages.reserve(2 * stitches.size()); //just how much storage can there actually be?
    steps.reserve(stitches.size());

    //get the out loop for a given 'in':
    auto source_loop = [&](uint32_t si, uint32_t ii) {
        assert(stitches[si].in[ii] < stitches.size());
        Loop ret(stitches[si].in[ii], 0);
        if (stitches[ret.stitch].out[ret.idx] != si) ret.idx = 1;
        assert(stitches[ret.stitch].out[ret.idx] == si);
        return ret;
    };

	// TODO BEN: Possible Scheduler Improvement - try to make passes
	// end with yarn-carrier maximally left or right

    { //build steps:
        //current yarn position w.r.t. active loops:
        struct YarnInfo {
            Loop loop = INVALID_LOOP;
            char direction = '\0';
        };
        std::map< uint32_t, YarnInfo > active_yarns;
        // this idx is into what ?
        std::map< Loop, StorageIdx > active_loops;


        uint32_t next_begin = 0;
        while (next_begin < stitches.size()) {
            //std::cout << "steps[" << steps.size() << "]:\n"; //DEBUG

            steps.emplace_back();
            Step &step = steps.back();

            { //figure out begin/end range for step:
                step.begin = next_begin;
                step.end = step.begin + 1;
                //steps take as many stitches as they can, given some constraints:
                uint32_t increases = 0, decreases = 0;
                uint32_t last_shaping = step.begin;
                if (stitches[step.begin].type == Stitch::Increase) {
                    ++increases;
                }
                if (stitches[step.begin].type == Stitch::Decrease) {
                    ++decreases;
                }
                auto basic_type = [](Stitch const &s) {
                    if (s.type == Stitch::Start) return '+';
                    else if (s.type == Stitch::End) return '-';
                    //else if(s.type == Stitch::Decrease) return '^';

                    else return '|';
                };

				// BEN ADDITION - KEEP TRACK OF # OF BASIC TYPES!
				int num_basics = 0;
				if (stitches[step.begin].data.is_basic_type) {
					++num_basics;
				}

                while (step.end < stitches.size()) {
                    //same basic type:
                    if (basic_type(stitches[step.end]) != basic_type(stitches[step.begin])) break;
                    // config asks for basic type, expects input to respect all other constraints
                    if(stitches[step.end].data.is_basic_type != stitches[step.begin].data.is_basic_type) break;

                    if(stitches[step.end].data.is_basic_type &&
                            stitches[step.end].data.id != stitches[step.begin].data.id) break;

					// BEN ADDITION - CASTING ON SEGMENTS SHOULD BE THEIR OWN STEP
					/*if ((stitches[step.begin].in[0] == -1U) != (stitches[step.end].in[0] == -1U)) {
						break;
					}*/

					/*
					if (step.end + 1 < stitches.size() && stitches[step.end].in[0] != -1U && stitches[step.end + 1].in[0] == -1U) {
						break;
					}
					*/

					// BEN ADDITION - DON'T ALLOW MULTIPLE BASIC TYPES!
					if (stitches[step.end].data.is_basic_type) {
						++num_basics;
					}
					if (num_basics > 1) break;

                    //cable info (all non cable stitches have a cable id -1):
                    //should be dealt with basic types
                    //This means cables can't participate over increases and decreases for now which is wierd
                    //if ((stitches[step.end].cable_info.cable_id) != (stitches[step.begin].cable_info.cable_id)) {
                    //    std::cout << "breaking step because cable index changed. " << step.begin << " to " << step.end << std::endl;
                    //    break;
                    //}

                    //same direction:
                    //DEBUG
                    if (stitches[step.end].direction != stitches[step.begin].direction) break;
                    //same yarn:[STEPS are broken when yarn changes]
                    if (stitches[step.end].yarn != stitches[step.begin].yarn) break;
                    //same course: (not sure this is really needed, but it does mean that every loop ends up in a Storage at some point)
                    if (stitches[step.end].in[0] != -1U && stitches[step.end].in[0] >= step.begin) break;
                    if (stitches[step.end].in[1] != -1U && stitches[step.end].in[1] >= step.begin) break;

                    //todo this can potentially interact with the whole basic type business
                    //in which case respect the type and not the threshold
                    //not too many decreases/increases in a segment (...and, if there might be, cut things back a bit)
                    if (stitches[step.end].type == Stitch::Increase) {
                        ++increases;
                        if (increases >= 4 && !stitches[step.end].data.is_basic_type) {
                            step.end = (step.end - last_shaping + 1) / 2 + last_shaping;
                            break;
                        }
                        last_shaping = step.end;
                    }
                    if (stitches[step.end].type == Stitch::Decrease) {
                        ++decreases;
                        if (decreases >= 4 && !stitches[step.end].data.is_basic_type) {
                            step.end = (step.end - last_shaping + 1) / 2 + last_shaping;
                            break;
                        }
                        last_shaping = step.end;
                    }
                    ++step.end;
                }
                next_begin = step.end;
            }

            std::vector< Loop > in_chain;
            std::vector< Loop > out_chain;

            //stitches take in_chain -> out_chain:
            for (uint32_t si = step.begin; si != step.end; ++si) {
                for (uint32_t i = 0; i < 2; ++i) {
                    if (stitches[si].in[i] == -1U) continue;
                    Loop l = source_loop(si, i);
                    in_chain.emplace_back(l);
                }
                for (uint32_t o = 0; o < 2; ++o) {
                    if (stitches[si].out[o] == -1U) continue;
                    out_chain.emplace_back(si, o);
                }
            }

            if (stitches[step.begin].direction == Stitch::CW) {
                std::reverse(in_chain.begin(), in_chain.end());
                std::reverse(out_chain.begin(), out_chain.end());
            }

            /*
            { //DEBUG
                std::cout << "  in:";
                for (auto const &l : in_chain) {
                    std::cout << " " << l.to_string();
                }
                std::cout << '\n';
                std::cout << "  out:";
                for (auto const &l : out_chain) {
                    std::cout << " " << l.to_string();
                }
                std::cout << '\n';
                std::cout.flush();
            }*/

            // also has some not two yarns not active at the same time constraint
            // which needs to be changed.

            YarnInfo &yarn = active_yarns[stitches[step.begin].yarn];

            { //fill in step's 'in' array based on storages holding in_chain:
                std::set< StorageIdx > used;
                for (auto const &l : in_chain) {
                    auto f = active_loops.find(l);
                    assert(f != active_loops.end() && "stitches should depend only on active stuff");
                    used.insert(f->second);
                }
                if (yarn.loop != INVALID_LOOP) {
                    auto f = active_loops.find(yarn.loop);
                    assert(f != active_loops.end() && "active yarn should reference active stuff");
                    used.insert(f->second);
                }

                step.in.assign(used.begin(), used.end());
            }

            { //do some loop surgery to figure out 'out' storages:
                std::list< Storage > outs;
                for (auto storage_idx : step.in) {
                    outs.emplace_back(storages[storage_idx]);
                }

				// BEN:
				//     This function takes in two loops, and makes sure that they are next-to each other a->b in ccw
				//     order, in the storages. This may mean splitting up an existing storage, or merging two
				//     storages in order for them to be adjacent
				// Q:
				//     Doesn't merging or splitting loops invalidate the active_loops storage index for all of the
				//     other yarns?
                //flip and re-jigger yarn storages:
                auto link_ccw = [&outs](Loop const &a, Loop const &b) {
                    //std::cout << "Linking " << a.to_string() << " -> " << b.to_string() << std::endl; //DEBUG
                    std::list< Storage >::iterator sa = outs.end();
                    uint32_t la = -1U;
                    std::list< Storage >::iterator sb = outs.end();
                    uint32_t lb = -1U;

                    for (auto oi = outs.begin(); oi != outs.end(); ++oi) {
                        for (uint32_t li = 0; li < oi->size(); ++li) {
                            if ((*oi)[li] == a) {
                                assert(sa == outs.end());
                                sa = oi;
                                assert(la == -1U);
                                la = li;
                            }
                            if ((*oi)[li] == b) {
                                assert(sb == outs.end());
                                sb = oi;
                                assert(lb == -1U);
                                lb = li;
                            }
                        }
                    }
                    assert(sa != outs.end() && la < sa->size());
                    assert(sb != outs.end() && lb < sb->size());

                    if (sa == sb) {
                        if ((la + 1) % sa->size() == lb) {
                            //already ccw! great.
                        } else {
                            //must split loop.
                            Storage non_ab;
                            Storage ab;
                            if (la < lb) {
                                ab.insert(ab.end(), sa->begin(), sa->begin() + la + 1);
                                non_ab.insert(non_ab.end(), sa->begin() + la + 1, sa->begin() + lb);
                                ab.insert(ab.end(), sa->begin() + lb, sa->end());
                            } else { assert(lb < la);
                                non_ab.insert(non_ab.end(), sa->begin(), sa->begin() + lb);
                                ab.insert(ab.end(), sa->begin() + lb, sa->begin() + la + 1);
                                non_ab.insert(non_ab.end(), sa->begin() + la + 1, sa->end());
                            }
                            //PARANOIA:
                            assert(non_ab.size() + ab.size() == sa->size());
                            la = -1U;
                            lb = -1U;
                            for (uint32_t li = 0; li < ab.size(); ++li) {
                                if (ab[li] == a) {
                                    assert(la == -1U);
                                    la = li;
                                }
                                if (ab[li] == b) {
                                    assert(lb == -1U);
                                    lb = li;
                                }
                            }
                            assert(la != -1U && lb != -1U);
                            assert((la + 1) % ab.size() == lb);
                            for (auto const &l : non_ab) {
                                assert(l != a && l != b);
                            }
                            //end PARANOIA
                            *sa = std::move(ab);
                            outs.emplace_back(non_ab);
                        }
                    } else {
                        //must merge loops
                        std::rotate(sa->begin(), sa->begin() + (la + 1), sa->end());
                        assert(sa->back() == a);
                        std::rotate(sb->begin(), sb->begin() + lb, sb->end());
                        assert(sb->front() == b);
                        //TODO: add_bridge(sa->front(), sa->back())
                        //TODO: add_bridge(sb->front(), sb->back())
                        sa->insert(sa->end(), sb->begin(), sb->end());
                        outs.erase(sb);
                    }

                };

                if (yarn.loop != INVALID_LOOP) {
                    if (yarn.direction != stitches[step.begin].direction) {
                        //reversing direction should happen on the same stitch:
                        assert(!in_chain.empty() && yarn.loop == (stitches[step.begin].direction == Stitch::CCW ? in_chain[0] : in_chain.back()));
                    } else {
                        assert(!in_chain.empty() && "Don't handle linking yarn into starts, though that might be useful.");


                        if (yarn.direction == Stitch::CCW) { //formed ccw, so yarn link is before first elt of chain:
                            link_ccw(yarn.loop, in_chain[0]);
                        } else {
                            link_ccw(in_chain.back(), yarn.loop);
                        }
                    }
                }

                for (uint32_t i = 0; i + 1 < in_chain.size(); ++i) {
                    link_ccw(in_chain[i], in_chain[i+1]);
                }

                //Now 'outs' should have in_chain in one ccw list.


                // --> replace with out_chain.
                if (in_chain.empty()) {
                    //empty in chain -> create a new cycle~

                    step.inter = Storage(); //inter is an empty cycle

                    Storage result;
                    result.insert(result.end(), out_chain.begin(), out_chain.end());
                    outs.emplace_front(result);
                } else {
                    //check that in_chain is indeed in order:
                    std::list< Storage >::iterator s = outs.end();
                    uint32_t l = -1U;

                    for (auto oi = outs.begin(); oi != outs.end(); ++oi) {
                        for (uint32_t li = 0; li < oi->size(); ++li) {
                            if ((*oi)[li] == in_chain[0]) {
                                assert(s == outs.end());
                                s = oi;
                                assert(l == -1U);
                                l = li;
                            }
                        }
                    }
                    assert(s != outs.end());
                    assert(l < s->size());
					// Ben: Put in_chain[0] at the back of the storage
                    std::rotate(s->begin(), s->begin() + l, s->end());

                    //move 's' to outs[0]:
                    std::swap(*outs.begin(), *s);
                    s = outs.begin();

                    //store the pre-transform version of outs[0] as the step's "inter":
                    step.inter = *s;

                    assert(s->size() >= in_chain.size());
                    for (uint32_t i = 0; i < in_chain.size(); ++i) {
                        assert((*s)[i] == in_chain[i]);
                    }

                    s->erase(s->begin(), s->begin() + in_chain.size());
                    s->insert(s->begin(), out_chain.begin(), out_chain.end());
                    if (s->empty()) {
                        outs.erase(s);
                        //TODO: special flag for steps whose inter doesn't actually turn into outs[0] ?
                    }
                }
                uint32_t base = storages.size();
                storages.insert(storages.end(), outs.begin(), outs.end());
                for (uint32_t i = base; i < storages.size(); ++i) {
                    step.out.push_back(i);
                }
            }

            /*{ //DEBUG: dump step ins/outs
                auto storage_to_string = [&](uint32_t idx) {
                    assert(idx < storages.size());
                    std::string ret = std::to_string(idx) + "[";
                    for (auto const &s : storages[idx]) {
                        if (&s != &storages[idx][0]) ret += ' ';
                        ret += s.to_string();
                    }
                    ret += "]";
                    return ret;
                };
                std::cout << "step[" << (&step - &steps[0]) << "]:\n";
                std::cout << "  in:";
                for (auto i : step.in) std::cout << ' ' << storage_to_string(i);
                std::cout << '\n';
                std::cout << " int:";
                for (auto const &l : step.inter) std::cout << ' ' << l.to_string();
                std::cout << '\n';
                std::cout << " out:";
                for (auto o : step.out) std::cout << ' ' << storage_to_string(o);
                std::cout << std::endl;
            }*/

            if (step.in.size() > 0 && step.out.size() > 0) {
                //make a map from step.inter -> step.out[0]
                std::vector< uint32_t > inter_to_out(step.inter.size(), -1U);

                std::map< Loop, CycleIndex > inter_loops;
                std::map< Loop, CycleIndex > out_loops;

                //std::cout << "step.out size = " << step.out.size() << std::endl;
                for (uint32_t oi = 0; oi < step.out.size(); ++oi) {
                    Storage const &inter = (oi == 0 ? step.inter : storages[step.out[oi]]);
                    for (uint32_t i = 0; i < inter.size(); ++i) {
                        auto const &l = inter[i];
                        auto ret = inter_loops.insert(std::make_pair(l, CycleIndex(oi, i)));
                        assert(ret.second);
                    }
                    Storage const &out = storages[step.out[oi]];
                    //std::cout << " out.size = " << out.size() << std::endl;
                    for (uint32_t i = 0; i < out.size(); ++i) {
                        auto const &l = out[i];
                        //std::cout << l.to_string() << " ";
                        auto ret = out_loops.insert(std::make_pair(l, CycleIndex(oi, i)));
                        assert(ret.second);
                    }
                   // std::cout << std::endl;
                }

                //First, the loops that actually change because of stitches:
                //Do this in a catch-all way, so that stitch type can be arbitrary
                {
                    // If a stitch is a wierd combination of mutliple sub-stitches
                    // that have a different loop to loop connection topology
                    // how stitch stores in and out has to change
                    //std::cout << "[step ="<<step.begin<<" to " << step.end << "]" << std::endl;
                    for (uint32_t si = step.begin; si != step.end; ++si) {
                        Stitch const &stitch = stitches[si];
                        // std::cout << "stitch: " << stitch.type << "[" <<si<<"], ";
                        // find out loops on out
                        auto t = out_loops.find(Loop(si,0));
                        // This fails if there is  a short-row right before the end cycle
                        // because it breaks the ending cycle into multiple pieces
                        // which don't have an out or something like that...
                        assert(t != out_loops.end());
                        assert(t->second.cycle == 0 && t->second.index < storages[step.out[0]].size());

                        // find source loops on inter
                        for (uint32_t i = 0; i < 2; ++i) {
                            if(stitch.in[i] == -1U) continue;
                            auto f = inter_loops.find(source_loop(si,i));
                            assert(f != inter_loops.end());
                            assert(f->second.cycle == 0 && f->second.index < inter_to_out.size());

                            assert(inter_to_out[f->second.index] == -1U);
                            inter_to_out[f->second.index] = t->second.index;
                        }
                    }

                }


                //add things in the cycle that aren't touched by stitches:
                for (uint32_t ii = 0; ii < step.inter.size(); ++ii) {
                    if (inter_to_out[ii] != -1U) continue;
                    auto t = out_loops.find(step.inter[ii]);
                    assert(t != out_loops.end());
                    assert(t->second.cycle == 0 && t->second.index < storages[step.out[0]].size());
                    inter_to_out[ii] = t->second.index;
                }

                step.inter_to_out = inter_to_out;
            }

            {
                 // either the increase goes along, or this is a bindoff
                if (step.end >= stitches.size() || stitches[step.end].yarn != stitches[step.end-1].yarn ||
                       stitches[step.end-1].out[0] == -1U ) {

                    auto f = active_yarns.find(stitches[step.begin].yarn);
                    assert(f != active_yarns.end());
                    active_yarns.erase(f);
                    // is this necessary ?
                    assert(active_yarns.empty()); //<-- should only have had one yarn active at a time anyway.
                } else {
                    // Update yarn position
                    yarn.loop = Loop(
                        step.end-1,
                        stitches[step.end-1].out[1] != -1U ? 1 : 0
                    );
                    assert(stitches[step.end-1].out[yarn.loop.idx] != -1U
                           );
                    yarn.direction = stitches[step.end-1].direction;
                }
            }

#if 0 // This needs to be active
            //more advanced yarn handling; not particularly well-handled
            { //update active yarn:
                if (out_chain.empty()) {
                    //no outs -> take yarn out.
                    auto f = active_yarns.find(stitches[step.begin].yarn);
                    assert(f != active_yarns.end());
                    active_yarns.erase(f);
                } else {
                    yarn.loop = Loop(
                        step.end-1,
                        stitches[step.end-1].out[1] != -1U ? 1 : 0
                    );
                    assert(stitches[step.end-1].out[yarn.loop.idx] != -1U);
                    yarn.direction = stitches[step.end-1].direction;
                }
            }
            { //update ~other ~ yarns:
                std::map< Loop, Loop > cw_out;
                std::map< Loop, Loop > ccw_out;
                for (uint32_t si = step.begin; si < step.end; ++si) {
                    Loop cw = Loop(si, -1U);
                    Loop ccw = Loop(si, -1U);
                    if (stitches[si].out[0] == -1U) {
                        //leave with '-1U' for out idx
                    } else if (stitches[si].out[1] == -1U) {
                        cw.idx = ccw.idx = 0;
                    } else {
                        if (stitches[si].direction == Stitch::CCW) {
                            cw.idx = 0;
                            ccw.idx = 1;
                        } else {
                            cw.idx = 1;
                            ccw.idx = 0;
                        }
                    }
                    for (uint32_t i = 0; i < 2; ++i) {
                        if (stitches[si].in[i] == -1U) continue;
                        Loop from(stitches[si].in[i], stitches[stitches[si].in[0]].find_out(si));
                        auto res = cw_out.insert(std::make_pair(from, cw));
                        assert(res.second);
                        res = ccw_out.insert(std::make_pair(from, ccw));
                        assert(res.second);
                    }

                    //TODO: (this would be a good place to update bridges as well)
                }
                for (auto &i_ay : active_yarns) {
                    auto &ay = i_ay.second;
                    if (ay.direction == Stitch::CCW) {
                        auto f = ccw_out.find(ay.loop);
                        if (f != ccw_out.end()) {
                            ay.loop = f->second;
                        }
                    } else {
                        auto f = cw_out.find(ay.loop);
                        if (f != cw_out.end()) {
                            ay.loop = f->second;
                        }
                    }
                    if (ay.loop.idx == -1U) {
                        //TODO: need to take yarn out, probably; does this ever come up though?
                        assert(false && "yarn-out case that probably never comes up");
                    }
                }
            }
#endif

            { //update active_loops:
                for (StorageIdx i : step.in) {
                    for (auto const &l : storages[i]) {
                        auto f = active_loops.find(l);
                        assert(f != active_loops.end());
                        active_loops.erase(f);
                    }
                }
                for (StorageIdx i : step.out) {
                    for (auto const &l : storages[i]) {
                        assert(!active_loops.count(l));
                        active_loops[l] = i;
                    }
                }
            }

        } //while (more stitches)

    } //end of build steps


    //Figure out possible shapes for storages near *interesting* steps:
    std::cout << "Figuring out shapes for interesting steps:" << std::endl; //DEBUG
    for (auto &step : steps) {
        //interesting steps have more than one out/in:
        if (step.in.size() <= 1 && step.out.size() <= 1) continue;


        std::cout << "\nsteps[" << (&step - &steps[0]) << "]:\n"; //DEBUG
        /*
        { //DEBUG
            for (StorageIdx s : step.in) {
                std::cout << "  uses storages[" << s << "]:";
                for (auto const &l : storages[s]) std::cout << " " << l.to_string();
                std::cout << "\n";
            }
            for (StorageIdx s : step.out) {
                std::cout << "  makes storages[" << s << "]:";
                for (auto const &l : storages[s]) std::cout << " " << l.to_string();
                std::cout << "\n";
            }
            std::cout.flush();
        }*/

        //Construction model:
        // input cycles come in some order and shape
        // (input cycles are perhaps reshaped) <-- "late reshape"
        // middle (intermediate) cycle is created
        // intermediate cycle is reshaped
        // knitting happens


        //get local copies of storages:
        std::vector< Storage > ins, outs, intermediates;
        for (auto si : step.in) {
            ins.emplace_back(storages[si]);
        }
        for (auto si : step.out) {
            outs.emplace_back(storages[si]);
        }
        intermediates.emplace_back(step.inter);
        for (uint32_t i = 1; i < step.out.size(); ++i) {
            intermediates.emplace_back(storages[step.out[i]]);
        }

        //The contents of 'inter + outs[1...]' and 'ins' should be the same. Construct a mapping:
        std::vector< std::vector< CycleIndex > > in_to_inter;
        {
            std::map< Loop, CycleIndex > loops;
            for (auto const &inter : intermediates) {
                for (uint32_t i = 0; i < inter.size(); ++i) {
                    auto const &l = inter[i];
                    auto ret = loops.insert(std::make_pair(l, CycleIndex(&inter - &intermediates[0], i)));
                    assert(ret.second);
                }
            }
            uint32_t used = 0;
            in_to_inter.reserve(ins.size());
            for (auto const &in : ins) {
                in_to_inter.emplace_back();
                in_to_inter.back().reserve(in.size());
                for (auto const &l : in) {
                    auto f = loops.find(l);
                    assert(f != loops.end());
                    in_to_inter.back().emplace_back(f->second);
                    ++used;
                }
                assert(in_to_inter.back().size() == in.size());
            }
            assert(in_to_inter.size() == ins.size());
            assert(used == loops.size());
        }

        //intermediates[0] (== step.inter) gets knit on and changes contents before it is output:
        assert(step.inter_to_out.size() == step.inter.size());

        //TODO: ~cache of transfer costs~ // meh


        //enumerate input orders and shapes:
        //  -- for each order, test if the intermediate storages are all cycles
        //     then figure out cost to reshape for knitting (for all possible reshapes)
        //     add [cost, input order, input shapes, output shapes, output order] to possible shapes list
        std::vector< uint32_t > remaining;
        remaining.reserve(ins.size());
        for (uint32_t i = 0; i < ins.size(); ++i) {
            remaining.emplace_back(i);
        }
        std::vector< uint32_t > in_order;
        std::vector< PackedShape > in_shapes(ins.size(), -1U);
        std::vector< CycleIndex > front;
        std::vector< CycleIndex > back;

        //Given a current order, test to see if it leaves intermediates in a valid configuration:
        auto test_order = [&]() -> const char * {
            struct Info {
                uint32_t back_min = -1U;
                uint32_t back_max = 0;
                uint32_t front_min = -1U;
                uint32_t front_max = 0;
                uint32_t zero_index = -1U;
                bool zero_on_back = false;
            };
            std::vector< Info > inter_info(intermediates.size());
            for (uint32_t i = 0; i < back.size(); ++i) {
                auto const &ci = back[i];
                if (ci.cycle == -1U) continue; //gap
                assert(ci.cycle < inter_info.size());
                auto &info = inter_info[ci.cycle];
                info.back_min = std::min(info.back_min, i);
                info.back_max = std::max(info.back_max, i);
                if (ci.index == 0) {
                    assert(info.zero_index == -1U);
                    info.zero_index = i;
                    info.zero_on_back = true;
                }
            }
            for (uint32_t i = 0; i < front.size(); ++i) {
                auto const &ci = front[i];
                if (ci.cycle == -1U) continue; //gap
                assert(ci.cycle < inter_info.size());
                auto &info = inter_info[ci.cycle];
                info.front_min = std::min(info.front_min, i);
                info.front_max = std::max(info.front_max, i);
                if (ci.index == 0) {
                    assert(info.zero_index == -1U);
                    info.zero_index = i;
                    info.zero_on_back = false;
                }
            }

            std::vector< PackedShape > inter_shapes;
            inter_shapes.reserve(intermediates.size());
            for (auto const &info : inter_info) {
                uint32_t cycle = &info - &inter_info[0];
                //check for gaps inside cycle:
                uint32_t count = 0;
                if (info.back_min <= info.back_max) count += (info.back_max - info.back_min + 1);
                if (info.front_min <= info.front_max) count += (info.front_max - info.front_min + 1);
                if (count > intermediates[cycle].size()) {
                    //gap found; bail out
                    return "gap";
                }
                assert(count == intermediates[cycle].size()); //must have seen all loops
                assert(info.zero_index != -1U); //including the zero element, of course

                Shape shape(0,0);

                //check that things are aligned:
                if (info.back_min > info.back_max) {
                    //only on front bed
                    assert(info.front_min <= info.front_max);
                    if (info.front_min != info.front_max) {
                        return "flat (front)";
                    }
                    shape.nibbles = Shape::BackLeft;
                    shape.roll = 0;
                } else if (info.front_min > info.front_max) {
                    //only on back bed
                    assert(info.back_min <= info.back_max);
                    if (info.back_min != info.back_max) {
                        return "flat (back)";
                    }
                    shape.nibbles = Shape::FrontLeft;
                    shape.roll = 0;
                } else {
                    //on both beds, so check alignment between beds:
                    if (std::abs(int32_t(info.back_min) - int32_t(info.front_min)) > 1) {
                        return "mis-aligned (left)";
                    }
                    if (std::abs(int32_t(info.back_max) - int32_t(info.front_max)) > 1) {
                        return "mis-aligned (right)";
                    }
                    shape.nibbles =
                          (info.back_min > info.front_min ? Shape::BackLeft : 0)
                        | (info.back_min < info.front_min ? Shape::FrontLeft : 0)
                        | (info.back_max < info.front_max ? Shape::BackRight : 0)
                        | (info.back_max > info.front_max ? Shape::FrontRight : 0)
                    ;
                    if (info.zero_on_back) {
                        shape.roll = (info.front_max - info.front_min + 1) + (info.back_max - info.zero_index);
                    } else {
                        shape.roll = info.zero_index - info.front_min;
                    }
                }

                std::vector< CycleIndex > data;
                data.reserve(intermediates[cycle].size());
                for (uint32_t i = 0; i < intermediates[cycle].size(); ++i) {
                    data.emplace_back(cycle, i);
                }
                std::vector< CycleIndex > expected_front, expected_back;
                shape.append_to_beds(data, CycleIndex(-1U, -1U), &expected_front, &expected_back);

                //DEBUG
                //std::cout << "    Testing:\n";
                //std::string front_string, back_string;
                //typeset_beds< CycleIndex >(expected_front, expected_back, [](CycleIndex const &ci){
                //	return ci.to_string();
                //}, " ", &front_string, &back_string);
                //std::cout << "      " << back_string << std::endl;
                //std::cout << "      " << front_string << std::endl;

                for (uint32_t i = info.back_min; i <= info.back_max; ++i) {
                    uint32_t r = i - std::min(info.back_min, info.front_min);
                    assert(r < expected_back.size());
                    if (back[i] != expected_back[r]) {
                        return "bad shape (back)";
                    }
                }
                for (uint32_t i = info.front_min; i <= info.front_max; ++i) {
                    uint32_t r = i - std::min(info.back_min, info.front_min);
                    assert(r < expected_front.size());
                    if (front[i] != expected_front[r]) {
                        return "bad shape (front)";
                    }
                }

                inter_shapes.emplace_back(shape.pack());
            }

            std::vector< uint32_t > out_order;
            out_order.reserve(outs.size());
            { //figure out order of outs:
                std::vector< bool > seen(outs.size(), false);
                auto do_out = [&seen, &out_order](uint32_t cycle) {
                    if (cycle == -1U) return;
                    assert(cycle < seen.size());
                    if (!seen[cycle]) {
                        seen[cycle] = true;
                        out_order.emplace_back(cycle);
                    }
                };
                for (uint32_t i = 0; i < std::max(front.size(), back.size()); ++i) {
                    if (i < front.size()) do_out(front[i].cycle);
                    if (i < back.size()) do_out(back[i].cycle);
                }
                //make sure we saw all out cycles:
                for (auto s : seen) {
                    assert(s);
                }
                assert(out_order.size() == outs.size());
            }


            //Order passed the test, now add possible transforms with their costs:

            //DEBUG:
            //std::cout << "Have a working order:" << std::endl;
            //std::string front_string, back_string;
            //typeset_beds< CycleIndex >(front, back, [](CycleIndex const &ci){
            //	return ci.to_string();
            //}, " ", &front_string, &back_string);
            //std::cout << "  " << back_string << std::endl;
            //std::cout << "  " << front_string << std::endl;

            { //new method: no baked-in transfers (will bake into edges instead):
                //so just penalize for the intermediate shapes (might be double-charging?):
                ScheduleCost cost;
                for (auto const &inter_shape : inter_shapes) {
                    cost += ScheduleCost::shape_cost(Shape::unpack(inter_shape));
                }



                ScheduleOption option;
                option.cost = cost;

                option.in_order = in_order;
                option.in_shapes = in_shapes;

                option.inter_shape = inter_shapes[0];

                option.out_order = out_order;
                option.out_shapes = inter_shapes;

                //HERE ALIGNMENT NEEDS TO BE FAVOURED?
                //out0 needs to be assigned a shape, so pick the cheapest one:
                std::vector< Shape > out0_shapes = Shape::make_shapes_for(outs[0].size());
                ScheduleCost best_cost = ScheduleCost::max();
                for (auto const &out0_shape : out0_shapes) {
                    ScheduleCost cost = ScheduleCost::shape_cost(out0_shape);
                    cost += ScheduleCost::transfer_cost(
                        intermediates[0].size(), Shape::unpack(inter_shapes[0]),
                        outs[0].size(), out0_shape,
                        step.inter_to_out);
                    if (cost < best_cost) {
                        best_cost = cost;
                        option.out_shapes[0] = out0_shape.pack();
                    }
                }

                step.options.emplace_back(option);
            }


            return nullptr;

        };

        std::function< void() > enumerate_orders = [&]() {
            if (remaining.empty()) {

                //DEBUG:
                //std::cout << "  Trying:\n";
                //std::string front_string, back_string;
                //typeset_beds< CycleIndex >(front, back, [](CycleIndex const &ci){
                //	return ci.to_string_simple();
                //}, "", &front_string, &back_string);
                //std::cout << "    " << back_string << std::endl;
                //std::cout << "    " << front_string << std::endl;



                test_order();

                //const char *reason =
                //if (reason != nullptr) { //DEBUG
                //	std::cout << "   FAILED: " << reason << "." << std::endl;
                //}
                return;
            }
            //order isn't done yet, try all shapes:
            for (uint32_t r = 0; r < remaining.size(); ++r) {
                std::swap(remaining[r], remaining.back());
                uint32_t idx = remaining.back();
                remaining.pop_back();
                in_order.emplace_back(idx);

                std::vector< Shape > shapes = Shape::make_shapes_for(ins[idx].size());
                for (auto const &shape : shapes) {
                    in_shapes[idx] = shape.pack();

                    uint32_t front_size_before = front.size();
                    uint32_t back_size_before = back.size();

                    shape.append_to_beds(in_to_inter[idx], CycleIndex(-1U, -1U), &front, &back);

                    enumerate_orders();

                    front.erase(front.begin() + front_size_before, front.end());
                    back.erase(back.begin() + back_size_before, back.end());

                    in_shapes[idx] = -1U;
                }

                assert(!in_order.empty() && in_order.back() == idx);
                in_order.pop_back();
                remaining.push_back(idx);
                std::swap(remaining[r], remaining.back());
            }
        };

        enumerate_orders();

        std::cout << "In total, step had " << step.options.size() << " scheduling options." << std::endl;

    } //interesting steps
    std::cout << "(done)" << std::endl;


    //Figure out possible shapes for storages near *boring* steps:
    std::cout << "Figuring out shapes for boring steps:" << std::endl; //DEBUG
    for (auto &step : steps) {
        if (!(step.in.size() <= 1 && step.out.size() <= 1) /* && step.in.size()!=0 && step.out.size() != 0*/) continue; //skip exciting steps

        uint32_t inter_roll = 0; //such that: storages[step.in[0]][i] == step.inter[i + inter_roll]
        //used to fix up inter_shape relative to in_shape so the stitches are in the same places.
        if (step.in.size() == 1) {
            assert(step.inter.size() == storages[step.in[0]].size());
            inter_roll = -1U;
            for (uint32_t roll = 0; roll < step.inter.size(); ++roll) {
                if (storages[step.in[0]][0] == step.inter[roll]) {
                    assert(inter_roll == -1U);
                    inter_roll = roll;
                }
            }
            assert(inter_roll != -1U);
            for (uint32_t i = 0; i < step.inter.size(); ++i) {
                auto a = storages[step.in[0]][i];
                auto b = step.inter[(i + inter_roll) % step.inter.size()];
                //std::cout << a.to_string() << " ==? " << b.to_string() << std::endl;
                assert(storages[step.in[0]][i] == step.inter[(i + inter_roll) % step.inter.size()]);
            }
        } else {
            assert(step.inter.empty());
        }

        if (step.in.size() == 0 && step.out.size() == 1) {
            //Starts a cycle -- can do so in any order.
            assert(stitches[step.begin].type == Stitch::Start);

            std::vector< Shape > out_shapes = Shape::make_shapes_for(storages[step.out[0]].size());
            ScheduleOption option;
            option.in_order.clear();
            option.in_shapes.clear();
            option.inter_shape = 0;
            option.out_order = {0};
            for (auto const &out_shape : out_shapes) {
                option.cost = ScheduleCost::shape_cost(out_shape);
                option.out_shapes = {out_shape.pack()};
                step.options.emplace_back(option);
            }

        } else if (step.in.size() == 1 && step.out.size() == 0) {
            //Ends a cycle -- can do so in any order.
            assert(stitches[step.begin].type == Stitch::End);

            std::vector< Shape > in_shapes = Shape::make_shapes_for(storages[step.in[0]].size());
            ScheduleOption option;
            option.out_order.clear();
            option.out_shapes.clear();
            option.in_order = {0};
            for (auto const &in_shape : in_shapes) {
                option.cost = ScheduleCost::shape_cost(in_shape);
                Shape inter_shape = in_shape;
                inter_shape.roll = (in_shape.roll + step.inter.size() - inter_roll) % step.inter.size();
                option.inter_shape = inter_shape.pack();
                option.in_shapes = {in_shape.pack()};

                //{ //DEBUG: in/inter shapes should match, right?
                //	std::vector< Loop > in_front, in_back, inter_front, inter_back;
                //	Shape::unpack(option.in_shapes[0]).append_to_beds(storages[step.in[0]], INVALID_LOOP, &in_front, &in_back);
                //	Shape::unpack(option.inter_shape).append_to_beds(step.inter, INVALID_LOOP, &inter_front, &inter_back);
                //	assert(in_front == inter_front && in_back == inter_back);
                //}

                step.options.emplace_back(option);
            }

        } else { assert(step.in.size() == 1 && step.out.size() == 1);

            // TODO if this step has to be on one bed only (between step.begin,step.end)
            std::vector< Shape > in_shapes = Shape::make_shapes_for(storages[step.in[0]].size());
            std::vector< Shape > out_shapes = Shape::make_shapes_for(storages[step.out[0]].size());
            assert(step.inter.size() == storages[step.in[0]].size());
            assert(step.inter.size() == step.inter_to_out.size());

            ScheduleOption option;
            option.in_order = {0};
            option.out_order = {0};
            int in_thresh = 10;
            int out_thresh = 10;
            for (auto const &in_shape : in_shapes) {
                option.in_shapes = {in_shape.pack()};
                Shape inter_shape = in_shape;
                inter_shape.roll = (in_shape.roll + step.inter.size() - inter_roll) % step.inter.size();
                option.inter_shape = inter_shape.pack();


                //Note: ------- new stuff----------
if(0){
                bool on_same_bed = true;
                {
                    std::map< Loop, uint32_t > loop_inds;
                    for (uint32_t i = 0; i < step.inter.size(); ++i) {
                        auto ret = loop_inds.insert(std::make_pair(step.inter[i], i));
                        assert(ret.second);
                    }
                    char last_bed = 'u';
                    std::cout<<"inter:";
                    for (uint32_t s = step.begin; s != step.end; ++s) {
                        auto const &st = stitches[s];
                        assert(st.in[0] != -1U );

                        Loop in0(st.in[0], stitches[st.in[0]].find_out(s));
                        assert(in0.idx == 0 || in0.idx == 1);
                        auto f0 = loop_inds.find(in0);
                        assert(f0 != loop_inds.end());

                        char bed;
                        int32_t needle;
                        inter_shape.size_index_to_bed_needle(step.inter.size(), f0->second, &bed, &needle);
                        std::cout<<(char)bed<<needle<<",";
                        if(last_bed == 'u'){
                            last_bed = bed;
                        }
                        else{
                            if(last_bed != bed){
                                on_same_bed = false;
                                break;
                            }
                        }

                        if(st.in[1] !=-1U){

                            Loop in0(st.in[1], stitches[st.in[1]].find_out(s));
                            assert(in0.idx == 0 || in0.idx == 1);
                            auto f0 = loop_inds.find(in0);
                            assert(f0 != loop_inds.end());

                            char bed;
                            int32_t needle;
                            inter_shape.size_index_to_bed_needle(step.inter.size(), f0->second, &bed, &needle);
                            std::cout<<(char)bed<<needle<<"*,";

                            assert(last_bed != 'u');
                            {
                                if(last_bed != bed){
                                    on_same_bed = false;
                                    break;
                                }
                            }

                        }
                    }
                    std::cout<<std::endl;
                }
                // if step wants things on the same bed and on_same_bed is not true, don't accept this configuration
                if(!on_same_bed &&stitches[step.begin].type != Stitch::Start && stitches[step.begin].type != Stitch::End) continue;
                else std::cout<<"<valid>"<<std::endl;
}
                // --------end of new stuff--------




                for (auto const &out_shape : out_shapes) {
                    option.out_shapes = {out_shape.pack()};

                    option.cost = ScheduleCost::shape_cost(in_shape); // TODO should this also be inter shape, should there be in->inter transfer??
                    option.cost += ScheduleCost::shape_cost(out_shape);
                    option.cost += ScheduleCost::transfer_cost(
                        step.inter.size(), inter_shape,
                        storages[step.out[0]].size(), out_shape,
                        step.inter_to_out
                    );

                    //{ //DEBUG: in/inter shapes should match, right?
                    //	std::vector< Loop > in_front, in_back, inter_front, inter_back;
                    //	Shape::unpack(option.in_shapes[0]).append_to_beds(storages[step.in[0]], INVALID_LOOP, &in_front, &in_back);
                    //	Shape::unpack(option.inter_shape).append_to_beds(step.inter, INVALID_LOOP, &inter_front, &inter_back);
                    //	assert(in_front == inter_front && in_back == inter_back);
                    //}

                    step.options.emplace_back(option);
                  //  if(!out_thresh) break;
                  //  out_thresh--;
                }
             //   if(!in_thresh) break;
             //   in_thresh--;
            }

        }
        std::cout << "steps[" << (&step - &steps[0]) << "] had " << step.options.size() << " scheduling options." << std::endl;
    }

    //---------------------
    // VN TODO: Figure out a way to input the upward planar embedding if that is coming from the user
    std::vector< int32_t > storage_positions(storages.size(), std::numeric_limits< int32_t >::max());
    std::vector< PackedShape > storage_shapes(storages.size(), -1U); //shapes of storages just after creation (should be same as the step_options of the step that made them)
    std::vector< uint32_t > step_options(steps.size(), -1U); //shapes of in and out cycles
    { //Next up: do upward-planar embedding.

        //For every 'interesting' step, build a DAGNode
        //For every chain of 1-in / 1-out steps, build a DAGEdge

        std::vector< DAGNode > nodes;
        std::vector< DAGEdge > edges;

        std::map< uint32_t, DAGEdgeIndex > active_edges; //maps from storages to edges currently in progress

        std::vector< uint32_t > node_steps;

        //dag edges corresponding to each step's ins/outs:
        std::vector< std::vector< DAGEdgeIndex > > step_in_edges;
        std::vector< std::vector< DAGEdgeIndex > > step_out_edges;
        step_in_edges.reserve(steps.size());
        step_out_edges.reserve(steps.size());

        std::cout << "Building DAG "; std::cout.flush();
        for (auto const &step : steps) {
            std::cout << "."; std::cout.flush();
            step_in_edges.emplace_back();
            step_out_edges.emplace_back();
            if (step.in.size() == 1 && step.out.size() == 1) {
                //1-1 step; accumulate DAGEdge cost matrix:
                auto f = active_edges.find(step.in[0]);
                assert(f != active_edges.end());
                assert(f->second < edges.size());
                DAGEdge &edge = edges[f->second];
                assert(edge.to == step.in[0]);
                active_edges.erase(f);

                std::vector< DAGCost > old_costs = std::move(edge.costs);
                uint32_t old_to_shapes = edge.to_shapes;
                assert(edge.to_shapes == Shape::count_shapes_for(storages[step.in[0]].size()));

                edge.to = step.out[0];
                edge.to_shapes = Shape::count_shapes_for(storages[step.out[0]].size());
                edge.costs.resize(edge.from_shapes * edge.to_shapes, ScheduleCost::max());

                //accumulate step costs into edge costs:
                for (auto const &option : step.options) {
                    assert(option.in_order.size() == 1 && option.in_order[0] == 0);
                    assert(option.in_shapes.size() == 1);
                    assert(option.out_order.size() == 1 && option.out_order[0] == 0);
                    assert(option.out_shapes.size() == 1);

                    uint32_t in_shape = Shape::unpack(option.in_shapes[0]).index_for(storages[step.in[0]].size());
                    uint32_t out_shape = Shape::unpack(option.out_shapes[0]).index_for(storages[step.out[0]].size());

                    for (uint32_t f = 0; f < edge.from_shapes; ++f) {
                        DAGCost &cost = edge.costs[f * edge.to_shapes + out_shape];
                        DAGCost test = old_costs[f * old_to_shapes + in_shape];
                        if (!(test == DAGCost::max())) {
                            test += option.cost;
                            cost = std::min(cost, test);
                        }
                    }
                }

                //step is <within> this edge:
                step_in_edges.back().emplace_back(&edge - &edges[0]);
                step_out_edges.back().emplace_back(&edge - &edges[0]);

                auto ret = active_edges.insert(std::make_pair(step.out[0], &edge - &edges[0]));
                assert(ret.second);
            } else { assert(step.in.size() != 1 || step.out.size() != 1);
                //Exciting step; build a DAGNode, consume some DAGEdges, start some DAGEdges.

                node_steps.emplace_back(&step - &steps[0]);
                nodes.emplace_back();
                DAGNode &node = nodes.back();

                std::vector< DAGEdgeIndex > in_edges;
                std::vector< DAGEdgeIndex > out_edges;

                //look up indices of in edges:
                for (uint32_t i = 0; i < step.in.size(); ++i) {
                    auto f = active_edges.find(step.in[i]);
                    assert(f != active_edges.end());
                    DAGEdge &edge = edges[f->second];
                    assert(edge.to == step.in[i]);
                    edge.to = &node - &nodes[0];
                    in_edges.emplace_back(f->second);
                    active_edges.erase(f);
                }

                //create out edges:
                for (uint32_t i = 0; i < step.out.size(); ++i) {
                    auto ret = active_edges.insert(std::make_pair(step.out[i], edges.size()));
                    assert(ret.second);
                    out_edges.emplace_back(edges.size());

                    edges.emplace_back();
                    DAGEdge &edge = edges.back();
                    edge.from = &node - &nodes[0];
                    edge.from_shapes = Shape::count_shapes_for(storages[step.out[i]].size());
                    edge.to = step.out[i];
                    edge.to_shapes = Shape::count_shapes_for(storages[step.out[i]].size());

                    edge.costs.assign(edge.from_shapes * edge.to_shapes, ScheduleCost::max());
                    assert(edge.from_shapes == edge.to_shapes);
                    if (step.out.size() > 1 || step.in.size() > 1) {
                        std::vector< Shape > shapes = Shape::make_shapes_for(storages[step.out[i]].size());
                        assert(shapes.size() == edge.from_shapes);
                        assert(shapes.size() == edge.to_shapes);
                        std::vector< uint32_t > map(storages[step.out[i]].size());
                        for (auto &m : map) {
                            m = &m - &map[0];
                        }
                        //interesting steps get a ("free") xfer pass on their output. Maybe all steps should? Hmm.
                        for (uint32_t f = 0; f < edge.from_shapes; ++f) {
                            for (uint32_t t = 0; t < edge.to_shapes; ++t) {
                                edge.costs[f * edge.to_shapes + t] = ScheduleCost::transfer_cost(
                                    storages[step.out[i]].size(), shapes[f],
                                    storages[step.out[i]].size(), shapes[t],
                                    map);
                            }
                        }
                    } else {
                        //NOTE: this ends up costing a bit of extra time, because multiple-source-shape edges always need a square cost matrix, but we don't actually have any costs to put into it, so we just make up a matrix that does not permit rotation:
                        for (uint32_t s = 0; s < edge.from_shapes; ++s) {
                            edge.costs[s * edge.to_shapes + s] = ScheduleCost::zero();
                        }
                    }
                }

                step_in_edges.back() = in_edges;
                step_out_edges.back() = out_edges;

                //PARANOIA: no duplicate in_edges or out_edges, right?
                assert(std::set< uint32_t >(in_edges.begin(), in_edges.end()).size() == in_edges.size());
                assert(std::set< uint32_t >(out_edges.begin(), out_edges.end()).size() == out_edges.size());

                node.options.reserve(step.options.size());
                for (auto const &option : step.options) {
                    node.options.emplace_back();
                    DAGOption &dag_option = node.options.back();
                    dag_option.cost = option.cost;
                    dag_option.in_order.reserve(step.in.size());
                    dag_option.in_shapes.reserve(step.in.size());
                    dag_option.out_order.reserve(step.out.size());
                    dag_option.out_shapes.reserve(step.out.size());
                    for (uint32_t i = 0; i < step.in.size(); ++i) {
                        uint32_t in = option.in_order[i];
                        assert(in < in_edges.size());
                        dag_option.in_order.emplace_back(in_edges[in]);
                        assert(in < step.in.size());
                        dag_option.in_shapes.emplace_back(Shape::unpack(option.in_shapes[in]).index_for(storages[step.in[in]].size()));
                        assert(dag_option.in_shapes.back() < edges[in_edges[in]].to_shapes); //DEBUG
                    }
                    assert(dag_option.in_order.size() == step.in.size());
                    assert(dag_option.in_shapes.size() == step.in.size());
                    for (uint32_t o = 0; o < step.out.size(); ++o) {
                        uint32_t out = option.out_order[o];
                        assert(out < out_edges.size());
                        dag_option.out_order.emplace_back(out_edges[out]);
                        assert(out < step.out.size());
                        assert(out < option.out_shapes.size());
                        dag_option.out_shapes.emplace_back(Shape::unpack(option.out_shapes[out]).index_for(storages[step.out[out]].size())); //<-- WHY IS THIS FAILING?
                        assert(dag_option.out_shapes.back() < edges[out_edges[out]].from_shapes); //DEBUG
                    }
                    assert(dag_option.out_order.size() == step.out.size());
                    assert(dag_option.out_shapes.size() == step.out.size());
                    //PARANOIA: no duplicates in order, right?
                    assert(std::set< uint32_t >(dag_option.in_order.begin(), dag_option.in_order.end()).size() == dag_option.in_order.size());
                    assert(std::set< uint32_t >(dag_option.out_order.begin(), dag_option.out_order.end()).size() == dag_option.out_order.size());
                }
                assert(node.options.size() == step.options.size());
            }
        } //for(steps)
        std::cout << " done." << std::endl;

        assert(node_steps.size() == nodes.size());

        std::cout << "Have " << edges.size() << " edges and " << nodes.size() << " nodes." << std::endl;

        std::vector< uint32_t > node_options;
        std::vector< int32_t > node_positions, edge_positions;

        if (!embed_DAG(nodes, edges, &node_options, &node_positions, &edge_positions, depth)) {
            std::cerr << "ERROR: failed to find an upward-planar embedding." << std::endl;
            return 1;
        }

        assert(node_options.size() == nodes.size());
        assert(node_positions.size() == nodes.size());
        assert(edge_positions.size() == edges.size());

        //Go through steps and assign selected option:
        assert(node_options.size() == nodes.size());
        for (uint32_t n = 0; n < nodes.size(); ++n) {
            assert(node_steps[n] < steps.size());
            std::cout << "Step " << node_steps[n] << " gets option " << node_options[n] << " of " << nodes[n].options.size() << " for cost " << nodes[n].options[node_options[n]].cost << std::endl;
            assert(node_options[n] < steps[node_steps[n]].options.size());
            assert(nodes[n].options.size() == steps[node_steps[n]].options.size());
            step_options[node_steps[n]] = node_options[n];
        }
        for (auto const &e : edges) {
            std::cout << "Edge " << (&e - &edges[0]) << " from " << e.from << " to " << e.to << " ends up with cost "; std::cout.flush();
            uint32_t from_shape = -1U;
            auto const &from_option = nodes[e.from].options[node_options[e.from]];
            for (uint32_t out = 0; out < from_option.out_order.size(); ++out) {
                if (from_option.out_order[out] == (&e - &edges[0])) {
                    assert(from_shape == -1U);
                    from_shape = from_option.out_shapes[out];
                }
            }
            uint32_t to_shape = -1U;
            auto const &to_option = nodes[e.to].options[node_options[e.to]];
            for (uint32_t in = 0; in < to_option.in_order.size(); ++in) {
                if (to_option.in_order[in] == (&e - &edges[0])) {
                    assert(to_shape == -1U);
                    to_shape = to_option.in_shapes[in];
                }
            }
            assert(from_shape != -1U);
            assert(to_shape != -1U);

            assert(from_shape < e.from_shapes);
            assert(to_shape < e.to_shapes);
            std::cout << e.costs[from_shape * e.to_shapes + to_shape] << std::endl;

        }

        //assign storage positions:
        for (auto const &step : steps) {
            uint32_t si = &step - &steps[0];
            if (step_in_edges[si].size() == 1 && step_out_edges[si].size() == 1) {
                assert(step_options[si] == -1U); //wasn't part of the dag nodes.
                assert(step_in_edges[si][0] == step_out_edges[si][0]);
            }
            assert(step_in_edges[si].size() == step.in.size());
            for (uint32_t in = 0; in < step.in.size(); ++in) {
                assert(step_in_edges[si][in] < edge_positions.size());
                assert(storage_positions[step.in[in]] == edge_positions[step_in_edges[si][in]]);
            }
            assert(step_out_edges[si].size() == step.out.size());
            for (uint32_t out = 0; out < step.out.size(); ++out) {
                assert(step_out_edges[si][out] < edge_positions.size());
                assert(storage_positions[step.out[out]] == std::numeric_limits< int32_t >::max());
                storage_positions[step.out[out]] = edge_positions[step_out_edges[si][out]];
            }
        }

        //build up cost matrices to figure out all unassigned options:
        struct ShapeCost {
            ScheduleCost cost = ScheduleCost::max();
            uint32_t option = -1U; //option that cost comes via
        };
        std::vector< std::vector< ShapeCost > > storage_costs;
        storage_costs.reserve(storages.size());
        for (auto const &storage : storages) {
            storage_costs.emplace_back(Shape::count_shapes_for(storage.size()));
        }
        std::vector< uint32_t > storage_steps(storages.size(), -1U);
        for (auto const &step : steps) {
            for (auto o : step.out) {
                assert(o < storage_steps.size());
                assert(storage_steps[o] == -1U);
                storage_steps[o] = &step - &steps[0];
            }
        }
        for (auto const &step : steps) {
            uint32_t si = &step - &steps[0];
            if (step_options[si] != -1U) {
                //already-assigned option:
                auto const &option = step.options[step_options[si]];
                //chase back storage_costs chains from ins:
                for (uint32_t in = 0; in < step.in.size(); ++in) {
                    uint32_t storage = step.in[in];
                    uint32_t idx = Shape::unpack(option.in_shapes[in]).index_for(storages[step.in[in]].size());
                    while (true) {
                        uint32_t si = storage_steps[storage];
                        assert(si < steps.size());
                        if (step_options[si] != -1U) break; //finished
                        assert(steps[si].in.size() == 1 && steps[si].out.size() == 1);
                        assert(steps[si].out[0] == storage);

                        assert(!(storage_costs[storage][idx].cost == ScheduleCost::max()));
                        uint32_t opt = storage_costs[storage][idx].option;
                        assert(opt < steps[si].options.size());

                        uint32_t in_idx = Shape::unpack(steps[si].options[opt].in_shapes[0]).index_for(storages[steps[si].in[0]].size());
                        uint32_t out_idx = Shape::unpack(steps[si].options[opt].out_shapes[0]).index_for(storages[steps[si].out[0]].size());
                        assert(out_idx == idx);

                        step_options[si] = opt;

                        idx = in_idx;
                        storage = steps[si].in[0];
                    }
                }
                //start storage_costs chains from outs:
                for (uint32_t out = 0; out < step.out.size(); ++out) {
                    uint32_t idx = Shape::unpack(option.out_shapes[out]).index_for(storages[step.out[out]].size());
                    assert(idx < storage_costs[step.out[out]].size());
                    storage_costs[step.out[out]][idx].cost = ScheduleCost::zero();
                    storage_costs[step.out[out]][idx].option = step_options[si];
                }
            } else {
                //was part of an edge, need to propagate:
                assert(step.in.size() == 1 && step.out.size() == 1);

                for (auto const &option : step.options) {
                    assert(option.in_order.size() == 1);
                    assert(option.in_shapes.size() == 1);
                    assert(option.out_order.size() == 1);
                    assert(option.out_shapes.size() == 1);

                    uint32_t in_shape = Shape::unpack(option.in_shapes[0]).index_for(storages[step.in[0]].size());
                    uint32_t out_shape = Shape::unpack(option.out_shapes[0]).index_for(storages[step.out[0]].size());

                    assert(in_shape < storage_costs[step.in[0]].size());
                    ScheduleCost test = storage_costs[step.in[0]][in_shape].cost;
                    if (!(test == ScheduleCost::max())) {
                        test += option.cost;
                        assert(out_shape < storage_costs[step.out[0]].size());
                        if (test < storage_costs[step.out[0]][out_shape].cost) {
                            storage_costs[step.out[0]][out_shape].cost = test;
                            storage_costs[step.out[0]][out_shape].option = &option - &step.options[0];
                        }
                    }
                }
            }
        }


        //record shapes:
        uint32_t pre_xfers = 0;
        for (uint32_t si = 0; si < steps.size(); ++si) {
            std::cout << "Step " << si << " option " << step_options[si] << " says "; //DEBUG
            assert(step_options[si] < steps[si].options.size());
            auto const &option = steps[si].options[step_options[si]];
            for (uint32_t in = 0; in < steps[si].in.size(); ++in) {
                std::cout << " i" << steps[si].in[in] << "=" << option.in_shapes[in]; std::cout.flush(); //DEBUG
                assert(storage_shapes[steps[si].in[in]] != -1U);
                if (storage_shapes[steps[si].in[in]] != option.in_shapes[in]) {
                    ++pre_xfers;
                    std::cout << "*"; std::cout.flush(); //DEBUG
                }
            }
            for (uint32_t out = 0; out < steps[si].out.size(); ++out) {
                std::cout << " o" << steps[si].out[out] << "=" << option.out_shapes[out]; //DEBUG
                assert(storage_shapes[steps[si].out[out]] == -1U);
                storage_shapes[steps[si].out[out]] = option.out_shapes[out];
            }
            std::cout << std::endl; //DEBUG
            //PARANOIA: check order vs storage positions:
            for (uint32_t i = 1; i < option.in_order.size(); ++i) {
                assert(option.in_order[i-1] < steps[si].in.size());
                assert(option.in_order[i] < steps[si].in.size());
                assert(
                    storage_positions[steps[si].in[option.in_order[i-1]]]
                    < storage_positions[steps[si].in[option.in_order[i]]]
                );
            }
            for (uint32_t i = 1; i < steps[si].options[step_options[si]].out_order.size(); ++i) {
                assert(option.out_order[i-1] < steps[si].out.size());
                assert(option.out_order[i] < steps[si].out.size());
                assert(
                    storage_positions[steps[si].out[option.out_order[i-1]]]
                    < storage_positions[steps[si].out[option.out_order[i]]]
                );
            }
        }
        if (pre_xfers > 0) {
            std::cout << "NOTE: will need to reshape " << pre_xfers << " inputs before steps." << std::endl;
        }

        //TODO: PARANOIA: check that step option orders match recorded storage positions.

        auto summarize = [](Shape const &shape, uint32_t size, uint32_t mark) -> std::string {
            //shape as braille dots
            // e.g.  ⠲⠶⠷⠶
            std::vector< char > data(size, '.');
            if (mark < size) data[mark] = '*';
            std::vector< char > front, back;
            shape.append_to_beds(data, ' ', &front, &back);

            if (front.size() % 2 == 1) front.emplace_back(' ');
            if (back.size() % 2 == 1) back.emplace_back(' ');

            std::vector< uint8_t > dots(std::max(front.size(), back.size())/2, 0);
            for (uint32_t i = 0; i + 1 < front.size(); i += 2) {
                if (front[i]   == '.') dots[i/2] |= 0x4;
                if (front[i+1] == '.') dots[i/2] |= 0x20;
                if (front[i]   == '*') dots[i/2] |= 0x4 | 0x40;
                if (front[i+1] == '*') dots[i/2] |= 0x20 | 0x80;
            }
            for (uint32_t i = 0; i + 1 < back.size(); i += 2) {
                if (back[i]   == '.') dots[i/2] |= 0x2;
                if (back[i+1] == '.') dots[i/2] |= 0x10;
                if (back[i]   == '*') dots[i/2] |= 0x2 | 0x1;
                if (back[i+1] == '*') dots[i/2] |= 0x10 | 0x8;
            }

            std::string ret;
            for (auto d : dots) {
                uint16_t c = 0x2800 | d;
                ret += char(0xe0 | ((c >> 12) & 0x1f));
                ret += char(0x80 | ((c >> 6) & 0x3f));
                ret += char(0x80 | ((c) & 0x3f));
            }
            return ret;
        };

        //DEBUG: dump selections:
        /*
        for (auto const &step : steps) {
            uint32_t si = &step - &steps[0];
            std::cout << "Step " << si << " gets option " << step_options[si] << " of " << steps[si].options.size() << "\n";
            auto const &option = steps[si].options[step_options[si]];
            std::cout << "  takes";
            for (auto i : option.in_order) {
                uint32_t in = step.in[i];
                uint32_t mark = 0;
                for (uint32_t s = 1; s < storages[in].size(); ++s) {
                    if (storages[in][mark] < storages[in][s]) mark = s;
                }
                std::cout << " s" << in << " in shape " << storage_shapes[in] << " " << summarize(Shape::unpack(storage_shapes[in]), storages[in].size(), mark) << " in lane " << storage_positions[in] << ",";
            }
            std::cout << "\n";
            std::cout << "  makes";
            for (auto o : option.out_order) {
                uint32_t out = step.out[o];
                uint32_t mark = 0;
                for (uint32_t s = 1; s < storages[out].size(); ++s) {
                    if (storages[out][mark] < storages[out][s]) mark = s;
                }
                std::cout << " s" << out << " in shape " << storage_shapes[out] << " " << summarize(Shape::unpack(storage_shapes[out]), storages[out].size(), mark) << " in lane " << storage_positions[out] << ",";
            }
            std::cout << "\n";
            std::cout.flush();
        }*/
    }

    struct StorageLeft {
        StorageLeft(uint32_t storage_, int32_t left_) : storage(storage_), left(left_) { }
        uint32_t storage;
        int32_t left;
    };
    std::vector< std::vector< StorageLeft > > step_storages; //left-edge positions for active storages just after the step
    std::vector< uint32_t > storage_widths; //widths of each storage in their just-after-step shapes.

    { //Determine left-edge position for each active storage just after each step:

        //this will be handy; max width (over front/back bed) of each storage:
        storage_widths.reserve(storages.size());
        for (auto const &storage : storages) {
            Shape shape = Shape::unpack(storage_shapes[&storage-&storages[0]]);
            std::vector< char > stitches(storage.size(), '.');
            std::vector< char > front, back;
            shape.append_to_beds(stitches, ' ', &front, &back);
            storage_widths.emplace_back(std::max(front.size(), back.size()));
        }

        //Now, for each stitch, record:
        std::vector< std::vector< uint32_t > > active; //indices of active (sorted left-to-right)
        std::vector< std::vector< int32_t > > gaps; //spaces between active
        active.reserve(steps.size());
        gaps.reserve(steps.size());
        { //determine what is active and start with a non-intersecting layout:
            std::set< uint32_t > current;
            for (auto const &step : steps) {
                for (auto in : step.in) {
                    auto f = current.find(in);
                    assert(f != current.end());
                    current.erase(f);
                }
                for (auto out : step.out) {
                    auto ret = current.insert(out);
                    assert(ret.second);
                }
                std::vector< uint32_t > act(current.begin(), current.end());
                std::sort(act.begin(), act.end(), [&](uint32_t a, uint32_t b){
                    return storage_positions[a] < storage_positions[b];
                });
                gaps.emplace_back(std::max(int32_t(act.size())-1, 0), 0);
                active.emplace_back(std::move(act));
            }
            assert(current.empty());
        }

        //TODO: build penalties between active storages at each step by looking at stitch ancestors

        //TODO: some sort of actual optimization or something!
        for (uint32_t si = 0; si < active.size(); ++si) {
            gaps[si].assign(gaps[si].size(), 1); //just a 'lil space between everything
        }

        std::vector< std::vector< int32_t > > lefts; //left edge of each active storage
        lefts.reserve(steps.size());
        for (uint32_t si = 0; si < steps.size(); ++si) {
            std::vector< int32_t > left;
            if (!active[si].empty()) {
                left.emplace_back(0);
                for (uint32_t i = 0; i < gaps[si].size(); ++i) {
                    left.emplace_back(left.back() + storage_widths[active[si][i]] + gaps[si][i]);
                }
            }
            //TODO: shift storage left positions based on penalties
            lefts.emplace_back(std::move(left));
        }

        step_storages.reserve(steps.size());
        for (uint32_t si = 0; si < steps.size(); ++si) {
            assert(lefts[si].size() == active[si].size());
            step_storages.emplace_back();
            auto &step_storage = step_storages.back();
            step_storage.reserve(active[si].size());
            for (uint32_t i = 0; i < active[si].size(); ++i) {
                step_storage.emplace_back(active[si][i], lefts[si][i]);
            }
        }

        { //DEBUG: show where the steps sit on actual needles
            int32_t min_needle = std::numeric_limits< int32_t >::max();
            int32_t max_needle = std::numeric_limits< int32_t >::min();
            for (uint32_t si = 0; si < steps.size(); ++si) {
                if (active[si].empty()) continue;
                min_needle = std::min(min_needle, lefts[si][0]);
                max_needle = std::max(max_needle, lefts[si].back() + int32_t(storage_widths[active[si].back()])-1);
            }
            for (uint32_t si = 0; si < steps.size(); ++si) {
                std::string num = std::to_string(si);
                while (num.size() < 4) num = ' ' + num;
                std::cout << "after " << num << ": ";

                std::vector< bool > on_back(max_needle - min_needle + 1, false);
                std::vector< bool > on_front(max_needle - min_needle + 1, false);

                for (uint32_t a = 0; a < active[si].size(); ++a) {
                    Shape shape = Shape::unpack(storage_shapes[active[si][a]]);
                    std::vector< bool > data(storages[active[si][a]].size(), true);
                    std::vector< bool > back, front;
                    shape.append_to_beds(data, false, &front, &back);
                    for (uint32_t i = 0; i < back.size(); ++i) {
                        if (back[i]) {
                            assert(on_back[i + (lefts[si][a] - min_needle)] == false);
                            on_back[i + (lefts[si][a] - min_needle)] = true;
                        }
                    }
                    for (uint32_t i = 0; i < front.size(); ++i) {
                        if (front[i]) {
                            assert(on_front[i + (lefts[si][a] - min_needle)] == false);
                            on_front[i + (lefts[si][a] - min_needle)] = true;
                        }
                    }
                }

                if (on_front.size() % 2 == 1) on_front.push_back(false);
                if (on_back.size() % 2 == 1) on_back.push_back(false);

                std::vector< uint8_t > dots(std::max(on_front.size(), on_back.size())/2, 0);
                for (uint32_t i = 0; i + 1 < on_front.size(); i += 2) {
                    if (on_front[i]  ) dots[i/2] |= 0x4;
                    if (on_front[i+1]) dots[i/2] |= 0x20;
                    //if (front[i]   == '*') dots[i/2] |= 0x4 | 0x40;
                    //if (front[i+1] == '*') dots[i/2] |= 0x20 | 0x80;
                }
                for (uint32_t i = 0; i + 1 < on_back.size(); i += 2) {
                    if (on_back[i]  ) dots[i/2] |= 0x2;
                    if (on_back[i+1]) dots[i/2] |= 0x10;
                    //if (back[i]   == '*') dots[i/2] |= 0x2 | 0x1;
                    //if (back[i+1] == '*') dots[i/2] |= 0x10 | 0x8;
                }

                for (auto d : dots) {
                    uint16_t c = 0x2800 | d;
                    std::cout << char(0xe0 | ((c >> 12) & 0x1f));
                    std::cout << char(0x80 | ((c >> 6) & 0x3f));
                    std::cout << char(0x80 | ((c) & 0x3f));
                }

                std::cout << " |";
                for (auto l : lefts[si]) {
                    std::cout << " " << l;
                }

                std::cout << std::endl;
            }
        }

    }

    //--- Dump knitting instructions steps ----

    auto add_instr = [&](std::string const &instr) {
        instructions.emplace_back(instr);
        //std::cout << instr << std::endl; //DEBUG
    };
    add_instr("const dsl = require('dsl');");
    add_instr("let h = new dsl.Helpers;");

    //helper:
    auto typeset_bed_needle = [](char bed, int32_t needle) -> std::string {
        std::string ret;
        ret += '\'';
        static_assert(BedNeedle::Front == 'f' && BedNeedle::Back == 'b', "BedNeedle and our ad-hoc bed characters agree.");
        if (bed == 'f' || bed == 'b') ret += bed;
        else if (bed == BedNeedle::FrontSliders) ret += "fs";
        else if (bed == BedNeedle::BackSliders) ret += "bs";
        else assert(0 && "unhandled bed name");
        ret += std::to_string(needle);
        ret += '\'';
        return ret;
    };

    //keep track of every created loop:
    //std::unordered_multimap< BedNeedle, Loop > bn_to_loop; <-- someday, update this incrementally...
    std::map< Loop, BedNeedle > loop_to_bn;

    auto make_start = [&loop_to_bn](char bed0, int32_t needle0, Loop const &out0) {
        assert( bed0 == 'f' ||  bed0 == 'b');
        BedNeedle bn0(bed0 == 'f' ? BedNeedle::Front : BedNeedle::Back, needle0);
        for (auto const &lbn : loop_to_bn) {
            assert(lbn.second != bn0); //needle should be vacant (NOTE: inefficient check!)
        }
        auto ret = loop_to_bn.insert(std::make_pair(out0, bn0));
        assert(ret.second); //loop shouldn't be in map already!
    };

    auto make_end = [&loop_to_bn](char bed0, int32_t needle0, Loop const &in0) {
        assert( bed0 == 'f' ||  bed0 == 'b');

        BedNeedle bn0(bed0 == 'f' ? BedNeedle::Front : BedNeedle::Back, needle0);

        { //remove input loop:
            auto f = loop_to_bn.find(in0);
            assert(f != loop_to_bn.end());
            assert(f->second == bn0); //input to stitch should be on this needle
            loop_to_bn.erase(f);
        }

        for (auto const &lbn : loop_to_bn) {
            assert(lbn.second != bn0); //needle should be vacant now that input is removed (NOTE: inefficient check!)
        }
    };

    auto make_xfers = [&loop_to_bn](std::vector< BedNeedle > const &from, std::vector< BedNeedle > const &to, Storage const &loops) {
        assert(from.size() == loops.size());
        assert(to.size() == loops.size());
        for (uint32_t li = 0; li < loops.size(); ++li) {
            //std::cout << "xfer " << loops[li].to_string() << " at " << typeset_bed_needle(from[li].bed, from[li].needle) << " to " << typeset_bed_needle(to[li].bed, to[li].needle) << std::endl; //DEBUG

            auto f = loop_to_bn.find(loops[li]);
            assert(f != loop_to_bn.end());
            //if (f->second != from[li]) {
            //	std::cout << "  is at " << typeset_bed_needle(f->second.bed, f->second.needle) << " instead?!?" << std::endl;
            //}
            assert(f->second == from[li]);
            loop_to_bn.erase(f);


        }
        //TODO: check that range used during xfer planning is actually empty
        for (uint32_t li = 0; li < loops.size(); ++li) {
            auto ret = loop_to_bn.insert(std::make_pair(loops[li], to[li]));
            assert(ret.second);
        }

    };
    // make stitch, increase and decrease is done in-place, perhaps introduce a catch-all make_stitch with lists


    //helper: flop to one bed (returns bed flopped to, if no preference given)
    auto stash = [&add_instr,&loop_to_bn,&typeset_bed_needle,&make_xfers](Storage const &storage, char const bed = '\0', int32_t offset = 0) -> char { //n.b. nonzero offset only makes sense when already stashed on *other* bed (maybe?)

        if (offset != 0) assert(bed != '\0');

        char target;
        {
            char stashed_to = '\0';
            for (auto const &loop : storage) {
                auto f = loop_to_bn.find(loop);
                assert(f != loop_to_bn.end());
                if (f->second.bed == BedNeedle::FrontSliders) {
                    if (stashed_to == '\0') stashed_to = 'f';
                    assert(stashed_to == 'f');
                }
                if (f->second.bed == BedNeedle::BackSliders) {
                    if (stashed_to == '\0') stashed_to = 'b';
                    assert(stashed_to == 'b');
                }
            }
            if (bed == '\0') {
                assert(offset == 0);
                target = (stashed_to != '\0' ? stashed_to : 'b');
            } else {
                if (offset != 0) {
                    assert(stashed_to != '\0' && stashed_to != bed);
                }
                target = bed;
            }
        }
        assert(target == 'f' || target == 'b');

        std::string from_str = "";
        std::string to_str = "";
        std::vector< BedNeedle > from_vec, to_vec;
        Storage loops;
        for (auto const &loop : storage) {
            auto f = loop_to_bn.find(loop);
            assert(f != loop_to_bn.end());

            BedNeedle from = f->second;
            BedNeedle to = f->second;

            if (target == 'b' && from.bed == BedNeedle::FrontSliders) to.bed = BedNeedle::Back;
            if (target == 'b' && from.bed == BedNeedle::Front) to.bed = BedNeedle::BackSliders;
            if (target == 'f' && from.bed == BedNeedle::BackSliders) to.bed = BedNeedle::Front;
            if (target == 'f' && from.bed == BedNeedle::Back) to.bed = BedNeedle::FrontSliders;

            to.needle += offset;

            if (from != to) {
                from_vec.emplace_back(from);
                to_vec.emplace_back(to);
                loops.emplace_back(loop);

                if (from_str != "") from_str += ", ";
                from_str += typeset_bed_needle(from.bed, from.needle);
                if (to_str != "") to_str += ", ";
                to_str += typeset_bed_needle(to.bed, to.needle);
            }
        }
        assert(from_vec.size() == to_vec.size());

        if (!from_vec.empty()) {
            std::string instr =
                "h.stash([" + from_str + "],\n"
                "        [" + to_str + "]);";

            add_instr(instr);

            make_xfers(from_vec, to_vec, loops);
        }
        return target;
    };

    //helper: flop from one bed
    auto unstash = [&add_instr,&loop_to_bn,&typeset_bed_needle,&make_xfers](Storage const &storage) {
        std::string from_str = "";
        std::string to_str = "";
        std::vector< BedNeedle > from, to;
        Storage loops;
        for (auto const &loop : storage) {
            auto f = loop_to_bn.find(loop);
            assert(f != loop_to_bn.end());
            if (f->second.bed == BedNeedle::FrontSliders || f->second.bed == BedNeedle::BackSliders) {
                from.emplace_back(f->second);
                to.emplace_back((f->second.bed == BedNeedle::FrontSliders ? BedNeedle::Back : BedNeedle::Front), f->second.needle);
                loops.emplace_back(loop);
                if (from_str != "") from_str += ", ";
                from_str += typeset_bed_needle(from.back().bed, from.back().needle);
                if (to_str != "") to_str += ", ";
                to_str += typeset_bed_needle(to.back().bed, to.back().needle);
            }
        }
        assert(from.size() == to.size());

        if (!from.empty()) {
            std::string instr =
            "h.unstash([" + from_str + "],\n"
            "          [" + to_str + "]);";

            add_instr(instr);

            make_xfers(from, to, loops);
        }
    };
    auto unstash_rec = [&unstash](Storage const &storage) {
        //TODO: should unstash any other storages that overlap this one <---
        unstash(storage);
    };

    //helper:
    auto reshape = [&add_instr,&stash,&unstash,&typeset_bed_needle,&make_xfers](std::unordered_map< Storage const *, std::pair< int32_t, Shape > > from_layout,
                                std::unordered_map< Storage const *, std::pair< int32_t, Shape > > const &to_layout) {
        //move things between different shapes:
        assert(from_layout.size() == to_layout.size()); //layouts should have the same elements

        //sort storages left-to-right:
        std::vector< Storage const * > active;
        for (auto const &sls : from_layout) {
            active.emplace_back(sls.first);
        }
        std::sort(active.begin(), active.end(), [&](Storage const *a, Storage const *b) {
            auto fa = from_layout.find(a);
            assert(fa != from_layout.end());
            auto fb = from_layout.find(b);
            assert(fb != from_layout.end());
            return fa->second.first < fb->second.first;
        });

        //check for anything that needs rolling:
        for (auto storage : active) {
            Shape *from_shape;
            int32_t *from_left;
            {
                auto f = from_layout.find(storage);
                assert(f != from_layout.end());
                from_left = &f->second.first;
                from_shape = &f->second.second;
            }
            Shape const *to_shape;
            int32_t const *to_left;
            {
                auto f = to_layout.find(storage);
                assert(f != to_layout.end());
                to_left = &f->second.first;
                to_shape = &f->second.second;
            }
            // Something awkward about these rolls...
            if (from_shape->pack() != to_shape->pack()) {
                //must roll the shape.
                {
                    uint32_t storage_idx = -1U;
                    for (uint32_t i = 0; i < active.size(); ++i) {
                        if (active[i] == storage) {
                            assert(storage_idx == -1U);
                            storage_idx = i;
                        }
                    }
                    assert(storage_idx != -1U);

                    int32_t front_min, front_max, back_min, back_max;
                    from_shape->size_to_range(storage->size(), &front_min, &front_max, &back_min, &back_max, *from_left);

                    //TODO: this would be improved a whole lot by having a stash function that knows about the order of storages, then it could do this recursively, which would be much cleaner!

                    { //stash stuff to the left:
                        char prev_bed = '\0';
                        int32_t prev_back_min = back_min;
                        int32_t prev_front_min = front_min;
                        for (uint32_t i = storage_idx - 1; i < active.size(); --i) {
                            Storage const *other = active[i];
                            assert(other != storage);
                            Shape const *other_shape;
                            int32_t const *other_left;
                            {
                                auto f = from_layout.find(other);
                                assert(f != from_layout.end());
                                other_left = &f->second.first;
                                other_shape = &f->second.second;
                            }
                            int32_t other_front_min, other_front_max, other_back_min, other_back_max;
                            other_shape->size_to_range(other->size(), &other_front_min, &other_front_max, &other_back_min, &other_back_max, *other_left);
                            assert(other_front_max < prev_front_min);
                            assert(other_back_max < prev_back_min);

                            char stash_bed;
                            if (other_front_max == prev_back_min) {
                                // o p p
                                // o o p
                                stash_bed = (prev_bed == 'b' ? '\0' : 'f');
                            } else if (other_back_max == prev_front_min) {
                                // o o p
                                // o p p
                                stash_bed = (prev_bed == 'f' ? '\0' : 'b');
                            } else {
                                stash_bed = '\0';
                            }
                            //add_instr("h.raw_inst(';stash stuff to the left');");

                            prev_bed = stash(*other, stash_bed);
                            prev_front_min = other_front_min;
                            prev_back_min = other_back_min;
                        }
                    }

                    { //stash stuff to the right:
                        char prev_bed = '\0';
                        int32_t prev_back_max = back_max;
                        int32_t prev_front_max = front_max;
                        for (uint32_t i = storage_idx - 1; i < active.size(); --i) {
                            Storage const *other = active[i];
                            assert(other != storage);
                            Shape const *other_shape;
                            int32_t const *other_left;
                            {
                                auto f = from_layout.find(other);
                                assert(f != from_layout.end());
                                other_left = &f->second.first;
                                other_shape = &f->second.second;
                            }
                            int32_t other_front_min, other_front_max, other_back_min, other_back_max;
                            other_shape->size_to_range(other->size(), &other_front_min, &other_front_max, &other_back_min, &other_back_max, *other_left);
                            assert(prev_front_max < other_front_min);
                            assert(prev_back_max < other_back_min);

                            char stash_bed;
                            if (prev_front_max == other_back_min) {
                                // p o o
                                // p p o
                                stash_bed = (prev_bed == 'b' ? '\0' : 'f');
                            } else if (prev_back_max == other_front_min) {
                                // p p o
                                // p o o
                                stash_bed = (prev_bed == 'f' ? '\0' : 'b');
                            } else {
                                stash_bed = '\0';
                            }
                            //add_instr("h.raw_inst(';stash stuff to the right');");


                            prev_bed = stash(*other, stash_bed);
                            prev_front_max = other_front_max;
                            prev_back_max = other_back_max;
                        }
                    }
                }

                //add_instr("h.raw_inst(';unstash before rolling');");

                unstash(*storage);

                std::string from_str = "";
                std::string to_str = "";
                std::vector< BedNeedle > from;
                std::vector< BedNeedle > to;

                for (uint32_t i = 0; i < storage->size(); ++i) {
                    char from_bed;
                    int32_t from_needle;
                    from_shape->size_index_to_bed_needle(storage->size(), i, &from_bed, &from_needle);
                    from_needle += *from_left;

                    assert(from_bed == 'f' || from_bed == 'b');

                    from.emplace_back(from_bed == 'f' ? BedNeedle::Front : BedNeedle::Back, from_needle);

                    if (i != 0) from_str += ", ";
                    from_str += typeset_bed_needle(from_bed, from_needle);

                    char to_bed;
                    int32_t to_needle;
                    to_shape->size_index_to_bed_needle(storage->size(), i, &to_bed, &to_needle);
                    to_needle += *to_left;

                    assert(to_bed =='f' || to_bed == 'b');

                    to.emplace_back(to_bed == 'f' ? BedNeedle::Front : BedNeedle::Back, to_needle);

                    if (i != 0) to_str += ", ";
                    to_str += typeset_bed_needle(to_bed, to_needle);
                }

                add_instr(
                    "h.roll_cycle([" + from_str + "]\n"
                    "             [" + to_str + "]);"
                );

                make_xfers(from, to, *storage);

                *from_shape = *to_shape;
            }
        }

        bool need_shift = false;
        for (auto storage : active) {
            Shape *from_shape;
            int32_t *from_left;
            {
                auto f = from_layout.find(storage);
                assert(f != from_layout.end());
                from_left = &f->second.first;
                from_shape = &f->second.second;
            }
            Shape const *to_shape;
            int32_t const *to_left;
            {
                auto f = to_layout.find(storage);
                assert(f != to_layout.end());
                to_left = &f->second.first;
                to_shape = &f->second.second;
            }
            assert(from_shape->pack() == to_shape->pack());

            if (*from_left != *to_left) {
                //there is a need to shift!
                std::cout << "needs shift because from_left = " << *from_left << " and to left = " << *to_left << std::endl;
                need_shift = true;
            }
        }

        char from_bed = 'b';
        char to_bed = 'f';

        if (need_shift) {
            //add_instr("h.raw_inst(';stash because shift is needed');");

            for (auto storage : active) {
                stash(*storage, from_bed);
            }
        }

        while (need_shift) {
            need_shift = false;

            //TODO: need to do this in the proper order to avoid loops getting snagged at irregular edges:
            for (auto storage : active) {
                Shape *from_shape;
                int32_t *from_left;
                {
                    auto f = from_layout.find(storage);
                    assert(f != from_layout.end());
                    from_left = &f->second.first;
                    from_shape = &f->second.second;

                }
                Shape const *to_shape;
                int32_t const *to_left;
                {
                    auto f = to_layout.find(storage);
                    assert(f != to_layout.end());
                    to_left = &f->second.first;
                    to_shape = &f->second.second;
                }
                assert(from_shape->pack() == to_shape->pack());

                int32_t offset = *to_left - *from_left;
                //racking limit (attempt!)
                if (offset <-2) offset = -2;
                if (offset > 2) offset =  2;

                //add_instr("h.raw_inst(';stashing for shifting by racking');");

                stash(*storage, to_bed, offset);

                *from_left += offset;

                if (*from_left != *to_left) {
                    //there is (still) a need to shift!
                    std::cout << "STILL needs shift because from_left = " << *from_left << " and to left = " << *to_left << std::endl;

                    need_shift = true;
                }
            }

            std::swap(from_bed, to_bed);
        }

    }; // reshape function

    std::unordered_map< Storage const *, std::pair< int32_t, Shape > > storage_layouts; //<-- is this really needed?
    for (uint32_t stepi = 0; stepi < steps.size(); ++stepi) {

        auto check_storage_layout = [&loop_to_bn](Storage const &storage, int32_t left, Shape const &shape) {
            bool front_stashed = false;
            bool front_unstashed = false;
            bool back_stashed = false;
            bool back_unstashed = false;

            for (uint32_t li = 0; li < storage.size(); ++li) {
                Loop const &loop = storage[li];
                char bed;
                int32_t needle;
                shape.size_index_to_bed_needle(storage.size(), li, &bed, &needle);
                needle += left;

                //std::cout << "Expecting " << loop.to_string() << " at " << typeset_bed_needle(bed, needle) << std::endl; //DEBUG

                auto fl = loop_to_bn.find(loop);
                assert(fl != loop_to_bn.end());
                assert(fl->second.needle == needle);
                if (bed == 'f') {
                    assert(fl->second.bed == BedNeedle::Front || fl->second.bed == BedNeedle::BackSliders);
                    if (fl->second.bed == BedNeedle::Front) front_unstashed = true;
                    if (fl->second.bed == BedNeedle::BackSliders) front_stashed = true;
                } else { assert(bed == 'b');
                    assert(fl->second.bed == BedNeedle::Back || fl->second.bed == BedNeedle::FrontSliders);
                    if (fl->second.bed == BedNeedle::Back) back_unstashed = true;
                    if (fl->second.bed == BedNeedle::FrontSliders) back_stashed = true;
                }
            }
            assert(!(front_stashed && front_unstashed));
            assert(!(back_stashed && back_unstashed));
            assert(!(front_stashed && back_stashed));
        };
        auto check_storage_layouts = [&step_storages,&storages,&storage_shapes,&storage_layouts,&check_storage_layout](uint32_t stepi) {
            //Check that locations in loop-tracking arrays are as expected:
            for (auto const &sl : step_storages[stepi]) {
                Storage const &storage = storages[sl.storage];
                auto f = storage_layouts.find(&storage);
                assert(f != storage_layouts.end());

                Shape shape = f->second.second;
                int32_t left = f->second.first;

                assert(shape.pack() == storage_shapes[sl.storage]);
                assert(left == sl.left);

                check_storage_layout(storage, left, shape);
            }
        };

        //std::cout << "Step[" << stepi << "]:" << std::endl; //DEBUG
        add_instr("//steps[" + std::to_string(stepi) + "]:");
        auto const &step = steps[stepi];
        assert(step_options[stepi] < step.options.size());
        auto const &option = step.options[step_options[stepi]];

        //partition storages into stuff to the left of the step and stuff to the right (useful if we need to shove):
        std::vector< uint32_t > left_of_step;
        std::vector< uint32_t > right_of_step;

        {
            //use 'storage_positions' which are abstract indices that yield a total order:
            int32_t step_min = std::numeric_limits< int32_t >::max();
            int32_t step_max = std::numeric_limits< int32_t >::min();
            for (auto si : step.in) {
                step_min = std::min(step_min, storage_positions[si]);
                step_max = std::max(step_max, storage_positions[si]);
            }
            for (auto si : step.out) {
                step_min = std::min(step_min, storage_positions[si]);
                step_max = std::max(step_max, storage_positions[si]);
            }
            assert(step_min <= step_max);

            std::set< uint32_t > in_step(step.in.begin(), step.in.end());
            for (auto const &sls : storage_layouts) {
                uint32_t idx = sls.first - &storages[0];
                assert(idx < storages.size()); //at this point, no inters should be in storages!
                if (in_step.count(idx)) continue;
                int32_t pos = storage_positions[idx];
                if (pos < step_min) {
                    left_of_step.emplace_back(idx);
                } else if (pos > step_max) {
                    right_of_step.emplace_back(idx);
                }
                else {
                    std::cout <<"pos " << pos << " step min " << step_min << " step max " << step_max << std::endl;
                    assert(pos <= step_min || pos >= step_max); //can't have another storage sitting in the middle of the step.
                }
            }
        }
        //DEBUG:
        /*
        std::cout << "  layout: ";
        for (auto si : left_of_step) std::cout << si << ' ';
        std::cout << '[';
        for (auto i : option.in_order) std::cout << (i == option.in_order[0] ? "" : " ") << step.in[i];
        std::cout << " -> ";
        for (auto o : option.out_order) std::cout << (o == option.out_order[0] ? "" : " ") << step.out[o];
        std::cout << ']';
        for (auto si : right_of_step) std::cout << ' ' << si;
        std::cout << std::endl;

        */
        //Compute the location of each loop in the step inputs:
        int32_t step_left = 0;
        std::vector< Loop > step_back;
        std::vector< Loop > step_front;

        //helper: will be useful for looking up shape locations in composite shape later:
        auto left_from_step = [&step_left, &step_back, &step_front](Shape const &shape, Storage const &loops) {
            //recomputing this is wasteful but keeps the other code cleaner:
            std::map< Loop, int32_t > step_needle;
            for (uint32_t i = 0; i < step_back.size(); ++i) {
                if (step_back[i] != INVALID_LOOP) {
                    auto ret = step_needle.insert(std::make_pair(step_back[i], step_left + i));
                    assert(ret.second);
                }
            }
            for (uint32_t i = 0; i < step_front.size(); ++i) {
                if (step_front[i] != INVALID_LOOP) {
                    auto ret = step_needle.insert(std::make_pair(step_front[i], step_left + i));
                    assert(ret.second);
                }
            }

            //use step_needle as lookup structure to figure out left:
            std::vector< Loop > front, back;
            shape.append_to_beds(loops, INVALID_LOOP, &front, &back);
            int32_t left = std::numeric_limits< int32_t >::max();
            for (uint32_t i = 0; i < back.size(); ++i) {
                if (back[i] == INVALID_LOOP) continue;
                auto l = step_needle.find(back[i]);
                assert(l != step_needle.end());
                int32_t pos = l->second - int32_t(i);
                if (left == std::numeric_limits< int32_t >::max()) left = pos;
                assert(left == pos);
            }
            for (uint32_t i = 0; i < front.size(); ++i) {
                if (front[i] == INVALID_LOOP) continue;
                auto l = step_needle.find(front[i]);
                assert(l != step_needle.end());
                int32_t pos = l->second - int32_t(i);
                if (left == std::numeric_limits< int32_t >::max()) left = pos;
                assert(left == pos);
            }
            return /*step_left +*/ left;
        };

        { //reshape to get ready for step -- shift everything to the correct offset and (maybe) also transform some things:
            auto old_layouts = storage_layouts;

            //move everything to the in shape: (most things shouldn't need it)
            assert(option.in_shapes.size() == step.in.size());
            for (uint32_t i = 0; i < step.in.size(); ++i) {
                auto f = storage_layouts.find(&storages[step.in[i]]);
                assert(f != storage_layouts.end());
                if (f->second.second.pack() != option.in_shapes[i]) {
                    std::cout << "  NOTE: late reshape for storage " << step.in[i] << std::endl; //DEBUG
                    f->second.second = Shape::unpack(option.in_shapes[i]); //<-- in some few cases, option's selected layout will be different from storage layout.
                }
            }

            //compute the composite shape:
            for (uint32_t i : option.in_order) {
                auto f = storage_layouts.find(&storages[step.in[i]]);
                assert(f != storage_layouts.end());
                f->second.second.append_to_beds(storages[step.in[i]], INVALID_LOOP, &step_front, &step_back);
            }

            /*
            { //DEBUG:
                std::string front_string, back_string;
                typeset_beds< class Loop >(step_front, step_back, [](Loop const &l){
                    return l.to_string();
                }, " ", &front_string, &back_string);
                std::cout << "  in: " << back_string << std::endl;
                std::cout << "      " << front_string << std::endl;
            }*/


            //figure out the left edge:
            //TODO: some sort of minimization to shift to the middle of the open space.
            if (!step.in.empty()) {
                auto f = storage_layouts.find(&storages[step.in[option.in_order[0]]]);
                assert(f != storage_layouts.end());
                step_left = f->second.first;
            }
            // std::cout << "step left " << step_left << std::endl;
            //reposition ins based on the chosen left edge:

            for (auto in : step.in) {
                auto f = storage_layouts.find(&storages[in]);
                assert(f != storage_layouts.end());
                int32_t new_left = left_from_step(f->second.second, storages[in]);
                assert(new_left != std::numeric_limits< int32_t >::max());
                f->second.first = new_left;
                //std::cout <<"\t new left for in based on the chosen left edge =" << new_left << std::endl;
            }

            // reshape before the step
            //add_instr("h.raw_inst(';reshape before the step');");

            reshape(old_layouts, storage_layouts);
        }


        { //reinterpret in[0..] as inter + outs[1...]:
            std::vector< Loop > inter_back;
            std::vector< Loop > inter_front;
            std::vector < Loop > out_back, out_front;
            if (step.out.empty()) {
                //special case: no outputs, so inter is *just* inter:
                Shape shape = Shape::unpack(option.inter_shape);
                shape.append_to_beds(step.inter, INVALID_LOOP, &inter_front, &inter_back);
            } else {
                for (auto o : option.out_order) {
                    if (o == 0) {
                        if (!step.inter.empty()) {
                            Shape shape = Shape::unpack(option.inter_shape);
                            shape.append_to_beds(step.inter, INVALID_LOOP, &inter_front, &inter_back);
                        }
                    } else {
                        Shape shape = Shape::unpack(option.out_shapes[o]);
                        // inter front back ??
                        shape.append_to_beds(storages[step.out[o]], INVALID_LOOP, &inter_front, &inter_back);
                        shape.append_to_beds(storages[step.out[o]], INVALID_LOOP, &out_front, &out_back);

                    }
                }
            }
            /*
            { //DEBUG:
                std::string front_string, back_string;
                typeset_beds< Loop >(inter_front, inter_back, [](Loop const &l){
                    return l.to_string();
                }, " ", &front_string, &back_string);
                std::cout << " int: " << back_string << std::endl;
                std::cout << "      " << front_string << std::endl;
            }
            { //DEBUG:
                std::string front_string, back_string;
                typeset_beds< Loop >(out_front, out_back, [](Loop const &l){
                    return l.to_string();
                }, " ", &front_string, &back_string);
                //std::cout << " *out: " << back_string << std::endl;
                //std::cout << "      " << front_string << std::endl;
            }*/

            assert(inter_back == step_back);
            assert(inter_front == step_front);

            //NOTE: if *not* unstashed, reinterpretation could lead to a mixed-stash cycle.
            for (auto in : step.in) {
                unstash(storages[in]);
            }

            //surgery on layouts to reflect the reinterpretation:
            for (uint32_t i = 0; i < step.in.size(); ++i) {
                auto f = storage_layouts.find(&storages[step.in[i]]);
                assert(f != storage_layouts.end());
                storage_layouts.erase(f);
            }

            if (!step.inter.empty()) {
                int32_t left = left_from_step(Shape::unpack(option.inter_shape), step.inter);
                auto ret = storage_layouts.insert(std::make_pair(&step.inter, std::make_pair(left, Shape::unpack(option.inter_shape))));
                assert(ret.second);
            }
            for (uint32_t o = 1; o < step.out.size(); ++o) {
                int32_t left = left_from_step(Shape::unpack(option.out_shapes[o]), storages[step.out[o]]);
                auto ret = storage_layouts.insert(std::make_pair(&storages[step.out[o]], std::make_pair(left, Shape::unpack(option.out_shapes[o]))));
                assert(ret.second);
            }

        }

        auto typeset_mapped_yarns = [&](const int yarn)->std::string{
          std::string ret = "\'";
          assert( yarn_mappings.count(yarn) && "yarn must be in mappings list.");
          auto pr = yarn_mappings[yarn];
          ret += std::to_string(pr.first);
          if(pr.second > 0 && pr.second != pr.first){
              ret += " " + std::to_string(pr.second);
          }
          ret += "\'";
          return ret;
        };

        auto do_yarn_handling = [&](const Stitch &curr){
            //std::cout <<"Doing yarn handling."<<std::endl;
            int id = &curr - &stitches[0];
            assert(id > 0);
            char bed = curr.bn_0.first;
            int needle = curr.bn_0.second;
            auto const &prev = stitches[id-1];
            std::string out_str = "h.take_carrier_out(" + typeset_mapped_yarns(curr.yarn) + ", " +
                    typeset_bed_needle(bed, needle) + ");";
            std::string in_str = "h.bring_carrier_in(" +  typeset_mapped_yarns(curr.yarn) + ", " + typeset_bed_needle(bed,needle) + ");";

            if(!curr.data.in_end){
                add_instr(out_str);
                active_carriers.erase(curr.yarn);
            }
            else {
                add_instr(in_str);
                active_carriers.insert(curr.yarn);
            }
        };
        auto do_switch_yarns = [&](const Stitch &prev, const Stitch &curr){
            //std::cout << "Trying to switch yarns." << std::endl;

            return;
            assert(active_carriers.count(prev.yarn));
            // look ahead for next use of yarn
            // if it is used in  less than some threshold count, just hold on to it
            int id = &curr - &stitches[0];
            bool keep_yarn = (curr.data.id != STITCH_ID_YARNEND);

            char bed = curr.bn_0.first;
            int needle = curr.bn_0.second;
            assert(yarn_mappings.count(prev.yarn));
            assert(yarn_mappings.count(curr.yarn));

            std::string out_str = "h.take_carrier_out(" + typeset_mapped_yarns(prev.yarn) + ", " +
                    typeset_bed_needle(bed, needle) + ");";
            std::string in_str = "h.bring_carrier_in(" +  typeset_mapped_yarns(curr.yarn) + ", " + typeset_bed_needle(bed,needle) + ");";


            if(!keep_yarn ){
            //    std::cout <<"Taking out carriers. " << std::endl;
                if(prev.type != Stitch::End)
                    add_instr(out_str); //taken out by end-tube
                active_carriers.erase(prev.yarn);
            }
            else{
            //    std::cout<<"Switching carriers, but not taking out yarn since it will be used in less than " << carrier_threshold << " stitches."<<std::endl;
            }
            if(!active_carriers.count(curr.yarn)){
           //     std::cout <<"Bringing in carrier. " << std::endl;
                add_instr(in_str);
                active_carriers.insert(curr.yarn);
            }
            else{
            //    std::cout <<"Switching carriers, but this is still available so not inserting yarn." << std::endl;
            }
        };
        //actually get around to doing the step:
        if (stitches[step.begin].type == Stitch::Start) {
            //cast-ons should be all cast-on:
            for (uint32_t s = step.begin; s != step.end; ++s) {
                assert(stitches[s].type == Stitch::Start);
            }

            assert(step.inter.empty());
            assert(step.out.size() == 1);
            assert(left_of_step.size() + right_of_step.size() == storage_layouts.size()); //don't have layout for out[0] stored yet.

            { //move everything else left/right to post-step locations:
                auto old_layouts = storage_layouts;
                for (uint32_t s : left_of_step) {
                    auto f = storage_layouts.find(&storages[s]);
                    assert(f != storage_layouts.end());
                    bool found = false;
                    for (auto const &sl : step_storages[stepi]) {
                        if (sl.storage == s) {
                            assert(!found);
                            found = true;
                            f->second.first = sl.left;
                        }
                    }
                }
                for (uint32_t s : right_of_step) {
                    auto f = storage_layouts.find(&storages[s]);
                    assert(f != storage_layouts.end());
                    bool found = false;
                    for (auto const &sl : step_storages[stepi]) {
                        if (sl.storage == s) {
                            assert(!found);
                            found = true;
                            f->second.first = sl.left;
                        }
                    }
                }
                // reshape for start
                // add_instr("h.raw_inst(';reshape for start');");

                reshape(old_layouts, storage_layouts);
            }

            //actually knit a cast-on:
            { //PARANOIA: does shape-to-needle stuff actually work?
                assert(!storages[step.out[0]].empty());
                uint32_t size = storages[step.out[0]].size();
                std::vector< Loop > out_back;
                std::vector< Loop > out_front;
                Shape shape = Shape::unpack(option.out_shapes[0]);
                shape.append_to_beds(storages[step.out[0]], INVALID_LOOP, &out_front, &out_back);
                for (uint32_t i = 0; i < size; ++i) {
                    char bed;
                    int32_t needle;
                    shape.size_index_to_bed_needle(size, i, &bed, &needle);
                    if (bed == 'f') {
                        assert(uint32_t(needle) < out_front.size());
                        assert(out_front[needle] == storages[step.out[0]][i]);
                    } else if (bed == 'b') {
                        assert(uint32_t(needle) < out_back.size());
                        assert(out_back[needle] == storages[step.out[0]][i]);
                    } else {
                        assert(bed == 'f' || bed == 'b');
                    }
                }
            }

            {
                Shape shape = Shape::unpack(option.out_shapes[0]);
                int32_t left = std::numeric_limits< int32_t >::max();
                for (auto const &sl : step_storages[stepi]) {
                    if (sl.storage == step.out[0]) {
                        assert(left == std::numeric_limits< int32_t >::max());
                        left = sl.left;
                    }
                }
                assert(left != std::numeric_limits< int32_t >::max());

                std::map< Loop, uint32_t > loop_inds;
                for (uint32_t o = 0; o < storages[step.out[0]].size(); ++o) {
                    auto ret = loop_inds.insert(std::make_pair(storages[step.out[0]][o], o));
                    assert(ret.second);
                }

                //TODO: need to unstash left/right if there are ragged edge overlaps
                //because cast-on would otherwise end up on the wrong side of adjacent tubes.

                // no longer necessary because start and end tubes bring in and take out
                // carriers in their helpers and the yarn-end stitch must be moved upwards/downwards
                // appropriately
                /*
                if(step.begin != 0){
                    auto prev = stitches[step.begin-1].yarn;
                    std::string inst = "h.take_carrier_out(";
                    inst += typeset_mapped_yarns(prev);
                    inst += " ,";
                    inst += typeset_bed_needle('f',0); // takeout doesn't really use the bed needle
                    inst += ");";
                    add_instr(inst);
                }*/

                active_carriers.insert(stitches[step.begin].yarn);
                std::string instr = "h.start_tube(";
                instr += (stitches[step.begin].direction == Stitch::Clockwise ? "'clockwise'" : "'anticlockwise'");
                instr += ", [";
                for (uint32_t s = step.begin; s != step.end; ++s) {
                    auto  &st = stitches[s];
                    st.step = stepi;
                    assert(st.type == Stitch::Start);
                    assert(st.out[0] != -1U && st.out[1] == -1U);

                    Loop out0(s, 0);
                    auto f0 = loop_inds.find(out0);
                    assert(f0 != loop_inds.end());

                    {//PARANOIA: should be exactly one loop here:
                        uint32_t on_needle = 0;
                        for (auto const &li : loop_inds) {
                            if (li.second == f0->second) ++on_needle;
                        }
                        assert(on_needle == 1);
                    }

                    char bed;
                    int32_t needle;
                    shape.size_index_to_bed_needle(storages[step.out[0]].size(), f0->second, &bed, &needle);
                    needle += left;

                    if (s != step.begin) instr += ", ";
                    instr += typeset_bed_needle(bed, needle);

                    make_start(bed, needle, Loop(s, 0));

                    st.bn_0 = std::make_pair(bed, needle);
                }
                instr += "], "+ typeset_mapped_yarns(stitches[step.begin].yarn) + ");"; // start-umbilical

                add_instr(instr);

                //add new tube to layouts:
                auto ret = storage_layouts.insert(std::make_pair(&storages[step.out[0]], std::make_pair(left, shape)));
                assert(ret.second);
            }

        } else if (stitches[step.begin].type == Stitch::End) {
            assert(step.out.empty());
            assert(step.in.size() == 1);

            Shape inter_shape = Shape::unpack(option.inter_shape);
            int32_t inter_left = left_from_step(inter_shape, step.inter);

            std::map< Loop, uint32_t > loop_inds;
            for (uint32_t i = 0; i < step.inter.size(); ++i) {
                auto ret = loop_inds.insert(std::make_pair(step.inter[i], i));
                assert(ret.second);
            }

            if(!active_carriers.count(stitches[step.begin].yarn)){
                {
                    // make sure st has the right bed needle
                    auto  &st = stitches[step.begin];
                    assert(st.type == Stitch::End);

                    Loop in0(st.in[0], stitches[st.in[0]].find_out(step.begin));
                    assert(in0.idx == 0 || in0.idx == 1);
                    auto f0 = loop_inds.find(in0);
                    assert(f0 != loop_inds.end());
                    char bed;
                    int32_t needle;
                    inter_shape.size_index_to_bed_needle(step.inter.size(), f0->second, &bed, &needle);
                    needle += inter_left;
                    st.bn_0 = std::make_pair(bed, needle);
                }
                do_switch_yarns(stitches[step.begin-1], stitches[step.begin]);
            }
            assert(active_carriers.count(stitches[step.begin].yarn));
            //int mapped_yarn = yarn_mappings[stitches[step.begin].yarn].first;
            std::string instr = "h.end_tube(";
            instr += (stitches[step.begin].direction == Stitch::Clockwise ? "'clockwise'" : "'anticlockwise'");
            instr += ", [";
            for (uint32_t s = step.begin; s != step.end; ++s) {
                auto  &st = stitches[s];
                st.step = stepi;
                assert(st.type == Stitch::End);
                assert(st.in[0] != -1U && st.in[1] == -1U);

                Loop in0(st.in[0], stitches[st.in[0]].find_out(s));
                assert(in0.idx == 0 || in0.idx == 1);
                auto f0 = loop_inds.find(in0);
                assert(f0 != loop_inds.end());

                {//PARANOIA: should be exactly one loop here:
                    uint32_t on_needle = 0;
                    for (auto const &li : loop_inds) {
                        if (li.second == f0->second) ++on_needle;
                    }
                    assert(on_needle == 1);
                }

                char bed;
                int32_t needle;
                inter_shape.size_index_to_bed_needle(step.inter.size(), f0->second, &bed, &needle);
                needle += inter_left;

                if (s != step.begin) instr += ", ";
                instr += typeset_bed_needle(bed, needle);

                make_end(bed, needle, in0);
                st.bn_0 = std::make_pair(bed, needle);
            }
            instr += "], "+  typeset_mapped_yarns(stitches[step.begin].yarn) + ");"; // end-umbilical


            add_instr(instr);

            { //no longer need to track inter layout; will replace with out[0]:
                auto f = storage_layouts.find(&step.inter);
                assert(f != storage_layouts.end());
                storage_layouts.erase(f);
            }

        } else {
            assert(step.out.size() >= 1);
            assert(step.in.size() >= 1);

            Shape inter_shape = Shape::unpack(option.inter_shape);
            int32_t inter_left = left_from_step(inter_shape, step.inter);

            Shape out_shape = Shape::unpack(option.out_shapes[0]);
            int32_t out_left = std::numeric_limits< int32_t >::max();
            for (auto const &sl : step_storages[stepi]) {
                if (sl.storage == step.out[0]) {
                    assert(out_left == std::numeric_limits< int32_t >::max());
                    out_left = sl.left;
                }
            }
            assert(out_left != std::numeric_limits< int32_t >::max());
            //std::cout <<"inter left =  " << inter_left << " out left = " << out_left << std::endl;
            //transform from inter_shape / inter_left to out_shape / out_left:
            std::string from_str, to_str;
            std::vector< BedNeedle > from, to;
            std::vector< BedNeedle > step_from, step_to;
            std::vector<int > firsts;
            bool need_xfer = false;
            for (uint32_t i = 0; i < step.inter.size(); ++i) {
                char from_bed;
                int32_t from_needle;
                inter_shape.size_index_to_bed_needle(step.inter.size(), i, &from_bed, &from_needle);

                auto const &il = step.inter[i];
                assert(il.stitch < stitches.size());
                auto from_st = stitches[il.stitch];
                assert(from_st.out[il.idx] != -1); // this stitch does produce the loop we care about
                from_needle += inter_left;


                uint32_t to_i = step.inter_to_out[i];
                assert(to_i < storages[step.out[0]].size());
                char to_bed;
                int32_t to_needle;
                out_shape.size_index_to_bed_needle(storages[step.out[0]].size(), to_i, &to_bed, &to_needle);
                to_needle += out_left;

                auto const &ol = storages[step.out[0]][to_i];
                auto to_st = stitches[ol.stitch];

                if (!from_str.empty()) from_str += ", ";
                from_str += typeset_bed_needle(from_bed, from_needle);

                if (!to_str.empty()) to_str += ", ";
                to_str += typeset_bed_needle(to_bed, to_needle);

                // TODO at some point, keep track of stitches behaviour through all subsequent steps until consumed
                if( to_st.step == -1){
                    /*
                    std::cout <<"from step: " << from_st.step << " to step: " << to_st.step << " curr step = " << stepi <<std::endl;
                    std::cout<<"Referring to loop "<<il.stitch<<","<<il.idx<<std::endl;
                    std::cout<<" Out loop " << ol.stitch <<","<<ol.idx<<std::endl;
                    std::cout << "output type " << to_st.type << stsd::endl;
                    std::cout<<" in type " << from_st.type << " is_d " << from_st.is_decreased_to << std::endl;
                    */
                    assert( from_bed == 'f' ||  from_bed == 'b');
                    assert( to_bed == 'f' ||  to_bed == 'b');

                    step_from.emplace_back(from_bed == 'f' ? BedNeedle::Front : BedNeedle::Back, from_needle);
                    step_to.emplace_back(to_bed == 'f' ? BedNeedle::Front : BedNeedle::Back, to_needle);


                }

                assert( from_bed == 'f' ||  from_bed == 'b');
                assert( to_bed == 'f' ||  to_bed == 'b');

                from.emplace_back(from_bed == 'f' ? BedNeedle::Front : BedNeedle::Back, from_needle);

                to.emplace_back(to_bed == 'f' ? BedNeedle::Front : BedNeedle::Back, to_needle);

                if (from_bed != to_bed || from_needle != to_needle) need_xfer = true;
            }
            // give space
            int32_t used_min = std::numeric_limits< int32_t >::max();
            int32_t used_max = std::numeric_limits< int32_t >::min();
            // other occupied
            int32_t left_max = std::numeric_limits< int32_t >::min();
            int32_t right_min = std::numeric_limits< int32_t >::max();
            // shift for space
            int32_t shift_left = 0;
            int32_t shift_right = 0;

            {
                //Figure out which needles are used during this xform:

                {
                    int32_t front_min, front_max, back_min, back_max;
                    inter_shape.size_to_range(step.inter.size(), &front_min, &front_max, &back_min, &back_max, inter_left);
                    used_min = std::min(used_min, std::min(front_min, back_min));
                    used_max = std::max(used_max, std::max(front_max, back_max));
                }
                {
                    int32_t front_min, front_max, back_min, back_max;
                    out_shape.size_to_range(storages[step.out[0]].size(), &front_min, &front_max, &back_min, &back_max, out_left);
                    used_min = std::min(used_min, std::min(front_min, back_min));
                    used_max = std::max(used_max, std::max(front_max, back_max));
                }
                //Figure out which other needles are occupied:

                bool on_right = false;
                for (auto const &sl : step_storages[stepi]) {
                    if (sl.storage == step.out[0]) {
                        assert(!on_right);
                        on_right = true;
                    } else {
                        { //make sure storage is the shape I expect:
                            auto f = storage_layouts.find(&storages[sl.storage]);
                            assert(f != storage_layouts.end());
                            //assert(f->second.first == sl.left); //not sure this will always be the case
                            assert(f->second.second.pack() == storage_shapes[sl.storage]);
                        }
                        int32_t front_min, front_max, back_min, back_max;
                        Shape::unpack(storage_shapes[sl.storage]).size_to_range(storages[sl.storage].size(), &front_min, &front_max, &back_min, &back_max, sl.left);
                        if (!on_right) {
                            left_max = std::max(left_max, std::max(front_max, back_max));
                        } else {
                            right_min = std::min(right_min, std::min(front_min, back_min));
                        }
                    }
                }

                if (left_max >= used_min) {
                    assert(left_max == used_min);
                    //need to move left at least one for some space:
                    shift_left -= -1;
                }
                if (used_max >= right_min) {
                    assert(used_max == right_min);
                    //need to move right at least one for some space:
                    shift_right += 1;
                }
                if ((left_max >= used_min - 1) && (used_max + 1 >= right_min)) {
                    //don't have a gap of one on at least one side
                    if (shift_left != 0) shift_left -= 1;
                    else shift_right += 1;
                }

                if (shift_left != 0 || shift_right != 0) {
                    auto old_layouts = storage_layouts;

                    bool on_right = false;
                    for (auto const &sl : step_storages[stepi]) {
                        if (sl.storage == step.out[0]) {
                            assert(!on_right);
                            on_right = true;
                        } else {
                            auto f = storage_layouts.find(&storages[sl.storage]);
                            assert(f != storage_layouts.end());

                            if (!on_right) {
                                f->second.first += shift_left;
                            } else {
                                f->second.first += shift_right;
                            }
                        }
                    }

                    // reshape for the actual step
                    // add_instr("h.raw_inst(';reshape before actual step');");
                    reshape(old_layouts, storage_layouts);
                }
                // add_instr("h.raw_inst(';unstash for step.inter ');");

                unstash_rec(step.inter);

            }

            // pre-xfer pass
            if(stitches[step.begin].data.is_custom_xfer)
            {
                // sanity check that pass was broken correctly
                for(uint32_t x = step.begin; x != step.end; x++){
                    assert(stitches[step.begin].data.id ==
                            stitches[x].data.id);
                    assert(stitches[x].data.is_custom_xfer);
                }
                //if there is a pre-block pass all these to st.func_name_pre_xfer_block
                //that takes the entire available unstashed block
                Constraints constraints;
                constraints.min_free = std::max(left_max + shift_left, used_min - 1);
                constraints.max_free = std::min(right_min + shift_right, used_max + 1);
                // a max racking of 4 + half-gauging can actually cause a max racking of 9
                constraints.max_racking = 2;
                // call pre-xfer pass on from,to, step_from, step_to, and constraints

                std::string instr ="";
                std::string func_name = "h."+stitches[step.begin].data.name + "_pre_xfer";
                instr += func_name + "([],[]";
                instr += ",[";
                for(auto bn : step_from){
                    instr += typeset_bed_needle(bn.bed, bn.needle) + ",";
                }
                instr += "],[";
                for(auto bn : step_to){
                    instr += typeset_bed_needle(bn.bed, bn.needle) + ",";
                }
                instr += "]";
                instr += ")";

                add_instr(instr);
            }
            if (need_xfer ) {
                //now do xfers:
                Constraints constraints;
                constraints.min_free = std::max(left_max + shift_left, used_min - 1);
                constraints.max_free = std::min(right_min + shift_right, used_max + 1);
                // a max racking of 4 + half-gauging can actually cause a max racking of 9
				constraints.max_racking = 2;

                std::vector< Slack > slack;
                slack.reserve(from.size());
                for (uint32_t i = 0; i < from.size(); ++i) {
                    Slack s = 1;
                    s = std::max(s, std::abs(from[i].needle - from[i+1<from.size()?i+1:0].needle));
                    s = std::max(s, std::abs(to[i].needle - to[i+1<to.size()?i+1:0].needle));
                    slack.emplace_back(s);
                }
                //for (uint32_t i = 0; i < from.size(); ++i) {
                //    std::cout<<"\t"<<from[i].to_string()<<" -> " <<to[i].to_string() << std::endl;
                //}
                std::string error = "";
                std::vector< Transfer > transfers;
                firsts.assign(from.size(),0);
                if(cse){
                    if (!plan_transfers(constraints, from, to, slack, &transfers, &error)) {
                        throw std::runtime_error("Failed to plan cycle transfer: " + error);
                    }
                }
                else{

                    if(!stacked_planner(constraints,from,to, firsts, slack,&transfers)){
                        throw std::runtime_error("Failed to plan cycle transfer with stacked planner. ");
                    }
                }

                // we planned some transfers anyway, but if this pass asks explicitly to not transfer plan
                // provide stitches[step.begin].func_name_xfer the froms, tos, step_froms, step_tos
                // we _will_ roll and shift to match storage, only shaping plans are optional


                std::string transfers_str;
                for (auto const &t : transfers) {
                    if (transfers_str != "") transfers_str += ", ";
                    transfers_str += "[" + typeset_bed_needle(t.from.bed, t.from.needle) + "," + typeset_bed_needle(t.to.bed, t.to.needle) + "]";
                }


                /*
                if(!step_from.empty()){
                    std::cout <<"pre-transfer here: #"<<step_from.size() <<" ::";
                    assert(step_from.size() == step_to.size());
                    // there is exactly one decrease that we are handling here
                    for(int i = 0; i < step_from.size(); i++){
                        std::cout<<(char)step_from[i].bed<<step_from[i].needle<<" > "
                        << (char)step_to[i].bed << step_to[i].needle <<" , ";
                    }
                    std::cout << std::endl;

                }*/

                // only emit this code if step does not explicitly avoid it:
                // NOTE always emitting this, custom will only locally re-arrange for now
                /*if(!stitches[step.begin].data.is_custom_xfer)*/
                {
                    std::string instr =
                            "h.xfer_cycle({minFree:" + std::to_string(constraints.min_free)
                            + ", maxFree:" + std::to_string(constraints.max_free)
                            + ", maxRacking:" + std::to_string(constraints.max_racking) + "},\n"
                            +"[" + from_str + "],\n"
                            +"[" + to_str + "],\n"
                            +"[" + transfers_str + "]);";                                                                                                                                                                                                                                    "             [" + transfers_str + "]);";

                    add_instr(instr);
                }



                //DEBUG
               // std::cout << "xfers \n"<<from_str<<" -> \n"<<to_str << std::endl;
                // make xfers either way, if the step did not ask for xfers, it does the same itself
                make_xfers(from, to, step.inter);

                // make sure you finally rack to 0, no ?
                add_instr("h.setRackingValue(0);");
            } // need_xfer


            //make sure inter is ready to knit:
            unstash_rec(step.inter); //this needs to recursively unstash neighbors


            // Make Stitches
            { //perform stitches:

                //record where each loop in 'inter' currently resides in the out shape:
                // (doing this because inter is in ccw order, and stitches may be in either direction)
                std::map< Loop, uint32_t > loop_inds;
                for (uint32_t i = 0; i < step.inter.size(); ++i) {
                    auto ret = loop_inds.insert(std::make_pair(step.inter[i], step.inter_to_out[i]));
                    assert(ret.second);
                }


                // the entire step has the same yarn so check for yarn switching before
                bool step_switches_yarn = false;
                if(step.begin > 0 && stitches[step.begin].yarn != stitches[step.begin-1].yarn){
                    step_switches_yarn = true;
                }


                add_instr("h.pass_emit();");
                for (uint32_t s = step.begin; s != step.end; ++s) {
                    auto  &st = stitches[s];
                    st.step = stepi;
                    // s should never be 0 here since a cast-on should exist

                    bool switch_yarns = false;
                    if(st.yarn != stitches[s-1].yarn){
                        switch_yarns  = true;
                    }
                    //TODO this stitch-number change will be performed in the javascript code
                    //if(s == step.begin && st.is_cable){
                    //    add_instr("h.set_stitch_number(h.CableStitchNumber);");
                    //}

                    //compute all the bed needles necessary based on the stitch type
                    //assumes face is at most a pentagon
                    {

                        // catch-all version
                        {
                            // find input stitches:
                            std::vector<BedNeedle> in_bns, out_bns;
                            std::vector<uint32_t> in_pos, out_pos;
                            std::vector<Loop> in_loops, out_loops;
                            for(uint32_t i = 0; i < 2 ; i++){ // this is always 2
                                if (st.in[i] == -1U) continue;
                                Loop in(st.in[i], stitches[st.in[i]].find_out(s));
                                auto f = loop_inds.find(in);
                                assert(f != loop_inds.end());
                                char bed;
                                int needle;
                                out_shape.size_index_to_bed_needle(
                                            storages[step.out[0]].size(),
                                            f->second, &bed, &needle
                                            );
                                needle += out_left;

                                BedNeedle bn;
                                bn.needle = needle;
                                if(bed == 'f'){
                                    bn.bed = BedNeedle::Front;
                                }
                                else if(bed == 'fs'){
                                    bn.bed = BedNeedle::FrontSliders;

                                }
                                else if(bed == 'b'){
                                    bn.bed = BedNeedle::Back;
                                }
                                else if(bed == 'bs'){
                                    bn.bed = BedNeedle::BackSliders;
                                }
                                else{
                                    assert(false && "unknown bed");
                                }

                                in_bns.push_back(bn);


                                in_pos.push_back(f->second);
                                in_loops.push_back(in);
                            }

                            // there are as many loops as collected, and they lie on the same position:
                            assert(in_pos.size()); // there is atleast one one loop
                            // same pos
                            for(uint32_t i = 1; i < in_pos.size(); i++){
                                assert(in_pos[i] == in_pos[0]);
                                assert(in_bns[i] == in_bns[0]); // the bed and needle positions also match, right
                            }
                            // as many as collected:
                            {
                                uint32_t n = 0;
                                for(auto const &li : loop_inds){
                                    if(li.second == in_pos[0]) n++;
                                }
                                assert(n == in_pos.size());
                            }

                            // what loops are created by this operation
                            for(uint32_t i = 0; i < 2; i++){
                                if(st.out[i] == -1U) continue;
                                Loop out(s, i);
                                out_loops.push_back(out);

                            }
                            {

                                out_pos.push_back(in_pos[0]);
                                char bed;
                                int32_t needle;
                                out_shape.size_index_to_bed_needle(storages[step.out[0]].size(), out_pos[0],
                                        &bed, &needle);
                                needle += out_left;
                                BedNeedle bn;
                                bn.needle = needle;
                                if(bed == 'f'){
                                    bn.bed = BedNeedle::Front;
                                }
                                else if(bed == 'fs'){
                                    bn.bed = BedNeedle::FrontSliders;

                                }
                                else if(bed == 'b'){
                                    bn.bed = BedNeedle::Back;
                                }
                                else if(bed == 'bs'){
                                    bn.bed = BedNeedle::BackSliders;
                                }
                                else{
                                    assert(false && "unknown bed");
                                }

                                out_bns.push_back(bn);
                            }

                            //out_bns.push_back(in_bns[0]); // first loop goes on the same place
                            //out_pos.push_back(in_pos[0]);
                            for(uint32_t i = 1; i < out_loops.size(); i++){
                                uint32_t idx1 = out_pos.back();
                                if (st.direction == Stitch::Clockwise) {
                                    idx1 = (idx1 > 0 ? idx1 - 1 : storages[step.out[0]].size()-1);
                                } else {
                                    idx1 = (idx1 + 1 < storages[step.out[0]].size() ? idx1 + 1 : 0);
                                }

                                {//no loop:
                                    for (auto const &li : loop_inds) {
                                        assert(li.second != idx1);
                                    }
                                }
                                out_pos.push_back(idx1);

                                char bed1;
                                int32_t needle1;
                                out_shape.size_index_to_bed_needle(storages[step.out[0]].size(), idx1, &bed1, &needle1);
                                needle1 += out_left;


                                BedNeedle bn;
                                bn.needle = needle1;
                                if(bed1 == 'f'){
                                    bn.bed = BedNeedle::Front;
                                }
                                else if(bed1 == 'fs'){
                                    bn.bed = BedNeedle::FrontSliders;

                                }
                                else if(bed1 == 'b'){
                                    bn.bed = BedNeedle::Back;
                                }
                                else if(bed1 == 'bs'){
                                    bn.bed = BedNeedle::BackSliders;
                                }
                                else{
                                    assert(false && "unknown bed");
                                }

                                out_bns.push_back(bn);

                            }

                            // remove old loops and make new loops:
                            //std::cout<<"All loops before removing: ";
                            //for(auto const &llbn : loop_to_bn){

                              //  std::cout<<typeset_bed_needle(llbn.second.bed,
                                //                              llbn.second.needle) <<",";
                            //}
                            //std::cout << std::endl;
                            // remove old:
                            //std::cout <<"Erasing loops: #"<<in_loops.size()<<" ";
                            for(uint32_t i = 0; i < in_loops.size(); i++){
                                auto f = loop_to_bn.find(in_loops[i]);
                                assert(f != loop_to_bn.end());
                              //  std::cout << typeset_bed_needle(f->second.bed,f->second.needle)<<",";
                                loop_to_bn.erase(f);




                            }
                            //std::cout<<std::endl;

                            //std::cout<<"All loops after removing: ";
                            //for(auto const &llbn : loop_to_bn){

                              //  std::cout<<typeset_bed_needle(llbn.second.bed,
                                //                              llbn.second.needle) <<",";
                            //}
                            //std::cout << std::endl;
                            // check that old bns don't exist
                            for(auto const &lbn : loop_to_bn){
                                for(uint32_t i = 0; i < in_bns.size(); i++){

                                    if(lbn.second == in_bns[i]){
                                        // print all the loops that exist:
                                        std::cout << "In loop that removed still exists in a different in location, did it get stashed there?" << std::endl;
                                        std::cout << "ERROR!!! This exists ? " << typeset_bed_needle(lbn.second.bed,lbn.second.needle)
                                                  << std::endl;
                                        std::cout <<std::flush;
                                        std::cout <<"BN: " << std::endl;
                                        for(auto x : loop_to_bn){
                                            std::cout << x.first.to_string() << "(" << typeset_bed_needle(x.second.bed, x.second.needle) << ") ,";

                                        }
                                        std::cout << std::endl;
                                    }

                                    //assert(lbn.second != in_bns[i]);
                                }
                            }
                            // insert new loops, make new directions while at it
                            assert(out_bns.size() == out_loops.size());
                            std::vector<std::string> directions;
                            for(uint32_t i = 0; i < out_bns.size(); i++){
                                auto ret = loop_to_bn.insert(std::make_pair(out_loops[i], out_bns[i]));
                                assert(ret.second); // was indeed inserted
                                std::string d;
                                if(st.direction == Stitch::Clockwise){
                                    d = (out_bns[i].bed == BedNeedle::Front ?"-":"+");
                                }
                                else{
                                    assert(st.direction == Stitch::Counterclockwise);
                                    d = (out_bns[i].bed == BedNeedle::Front ? "+":"-");

                                }
                                assert(d == "+" || d == "-");
                                directions.push_back(d);
                            }

                            assert(directions.size() == out_bns.size());

                            // update stitch with postiion inforrmation for display:
                            st.bn_0 = std::make_pair(out_bns[0].bed, out_bns[0].needle);

                            if(switch_yarns){
                                do_switch_yarns(stitches[s-1], st);
                            }
                            if(st.data.id == STITCH_ID_YARNEND && stitches[s-1].yarn != st.yarn){
                                // we either need to switch in or aout
                                do_yarn_handling(st);
                            }
                            std::string instr = ""; // function(dir_array, bn_array, yarns)
                            instr += "h." + st.data.name + "_make(";
                            instr += "[";
                            // javascript is fine with trailing commas:
                            for(auto d : directions){
                                instr += '\'' + d + '\'' + ',';
                            }
                            instr += "], [";
                            for(auto bn : out_bns){
                                instr += typeset_bed_needle(bn.bed,bn.needle) + ",";
                            }
                            instr += "],";
                            //assert(yarn_mappings.count(st.yarn));
                            //int mapped_yarn = yarn_mappings[st.yarn].first;
                            instr += typeset_mapped_yarns(st.yarn);
                            instr += ");";

                            add_instr(instr);

                            if(st.data.id == STITCH_ID_YARNEND && stitches[s-1].yarn == st.yarn){
                                // we either need to switch in or aout
                                do_yarn_handling(st);
                            }
                            // todo this should also be a list:
                            if(out_bns.size()>1){
                                st.bn_1 = std::make_pair(out_bns[1].bed,out_bns[1].needle);
                            }


                        }

                    }


                } // for each stitch
                add_instr("h.pass_commit();");


                //if(step_switches_yarn)
                {
                    //if there aren't any anchors nothing should happen,
                    // so just drop after every pass
                    // drop anchors at the end of this step, after committing pass
                    add_instr("h.drop_anchors();");
                }

                // if this step wanted break-pass:
                // these passes interact wierdly with increase-decrease?
                //std::cout<<"post-pass:";
                if(stitches[step.begin].data.is_basic_type)
                {
                    //post makes are all now face main types, the yarn here is virtual so set it to -1
                    // pass the entire pass to st.func_name_post_block here
                    std::vector<BedNeedle> step_bns;
                    std::vector<std::string> step_dirs;

                    for(uint32_t x = step.begin; x != step.end; x++){
                       auto const &st = stitches[x];
                       //std::cout <<st.data.name <<",";
                       assert(st.data.is_basic_type);
                       assert(st.bn_0.first != 'u');
                       BedNeedle b;
                       assert( st.bn_0.first == 'f' ||  st.bn_0.first == 'b');
                       b.bed = st.bn_0.first == 'f' ? BedNeedle::Front : BedNeedle::Back;
                       b.needle = st.bn_0.second;
                       step_bns.push_back(b);

                       std::string d;
                       if(st.direction == Stitch::Clockwise){
                           d = (b.bed == BedNeedle::Front ?"-":"+");
                       }
                       else{
                           assert(st.direction == Stitch::Counterclockwise);
                           d = (b.bed == BedNeedle::Front ? "+":"-");

                       }
                       assert(d == "+" || d == "-");
                       step_dirs.push_back(d);
                       if(st.bn_1.first != 'u'){
                           assert( st.bn_1.first == 'f' ||  st.bn_1.first == 'b');

                           BedNeedle b;
                           b.bed = st.bn_1.first == 'f' ? BedNeedle::Front : BedNeedle::Back;
                           b.needle = st.bn_1.second;
                           step_bns.push_back(b);
                       }
                    }

                    {
                    std::string instr = "h.";
                    instr += stitches[step.begin].data.name;
                    instr += "_post_make([";
                    for(auto bn : step_bns){
                        instr += typeset_bed_needle(bn.bed, bn.needle);
                        instr += ",";
                    }
                    instr +="]);";
                    add_instr(instr);
                    }

                    // if we wanted to call a single face program, do this:
                    // a new face has to be popped out in the ui so that it can
                    // actually change the associated code
                    /*

                    {
                        std::string instr = "h.";
                        instr += stitches[step.begin].data.name;
                        instr += "_make([";
                        for(auto d : step_dirs){
                            instr += d +",";
                        }
                        instr += "], [";
                        for(auto bn : step_bns){
                            instr += typeset_bed_needle(bn.bed, bn.needle);
                            instr += ",";
                        }
                        instr +="], -1);"; //or yarn is undefined
                        add_instr(instr);
                    }
                    */


                }
                //std::cout<<std::endl;
                if(false) // TODO FIX THIS BY READING THE RIGHT FILE
                {
                    // well the  entire step is a cable sequencet:
                    // TODO this has to be stacked_loop_connections
                    typedef std::pair<int,std::pair<int,int>> SLC;
                    std::vector<SLC > stacked_stitch_connections;

                    char start_bed = stitches[step.begin].bn_0.first;
                    bool same_bed = true;
                    bool reverse_locally = false;
                    if(stitches[step.begin].cable_info.local_id > stitches[step.end-1].cable_info.local_id){
                       reverse_locally = true;

                    }
                    if(reverse_locally){
                        for (uint32_t ss = step.begin; ss != step.end-1; ++ss){
                            assert(stitches[ss].cable_info.local_id > stitches[ss+1].cable_info.local_id);
                        }
                    }
                    else{
                        for (uint32_t ss = step.begin; ss != step.end-1; ++ss){
                            assert(stitches[ss].cable_info.local_id < stitches[ss+1].cable_info.local_id);
                        }
                    }

                    for (uint32_t ss = step.begin; ss != step.end; ++ss) {
                        auto const &st = stitches[ss];
                        int conn = st.cable_info.connection;
                        if(reverse_locally){
                            conn = (step.end - step.begin) - conn - 1;
                        }
                        //assert(st.is_cable && "cables are broken into their own steps");
                        assert(st.cable_info.cable_id == stitches[step.begin].cable_info.cable_id);
                        stacked_stitch_connections.push_back( std::make_pair(st.cable_info.stack, std::make_pair( ss, conn)));
                        std::cout <<"cable stitch " << ss << " goes to " << st.cable_info.connection << std::endl;
                        // TODO handle shaping case:

                        if(st.bn_0.first != start_bed) same_bed = false;
                    }
                    // atleast when not dealing with increases and decreases, right ?
                    assert(stacked_stitch_connections.size() == (step.end - step.begin));
                    // sort by stacking order
                    {
                        std::sort(stacked_stitch_connections.begin(), stacked_stitch_connections.end(), [](SLC &a, SLC &b)->bool{
                            return a.first < b.first;
                        });
                    }
                    // Transfers
                    // map cables to the class that makes sense or have a general approach
                    // stitches that are held on the bed will be affected by transfering for cables
                    // need to think about the case where the stitches are on different beds
                    if(same_bed)
                    {
                        // xfer all stitches to the other bed
                        char other_bed = (start_bed == 'f' ? 'b' : 'f');
                        for (uint32_t ss = step.begin; ss != step.end; ++ss) {
                                auto const &st = stitches[ss];
                                assert(st.bn_0.first == start_bed);
                                std::ostringstream str;
                                str <<"h.xfer( '" << start_bed << st.bn_0.second << "','" << other_bed <<'s' << st.bn_0.second <<"');";
                                //std::string const&  = "h.xfer("+ start_bed + std::to_string(st.bn_0.second) + "," +
                                //        std::to_string(other_bed)+"s" + std::to_string(st.bn_0.second) + ");";
                                std::cout <<"XFER STR : " << str.str() << std::endl;
                                add_instr(str.str());
                        }
                        for(int i = 0; i < stacked_stitch_connections.size(); i++){
                            const SLC &slc = stacked_stitch_connections[i];
                            assert(slc.first == i && "stack should be in order");
                            int s_from = slc.second.first;
                            int s_to = step.begin + slc.second.second;
                            assert(s_to < step.end && "connection within the same cable block");
                            // everything is on "other_bed"
                            //std::string const& rack_str =" h.setRacking(" + std::to_string(other_bed) + "s"+ std::to_string(stitches[s_from].bn_0.second)
                            //                                    + "," + std::to_string(start_bed) + std::to_string(stitches[s_to].bn_0.second) + ");";

                            std::ostringstream rack_str;
                            rack_str <<"h.setRacking( " << "'" << other_bed << 's' << stitches[s_from].bn_0.second << "' , '" << start_bed  << stitches[s_to].bn_0.second <<"' );";

                            add_instr(rack_str.str());

                            std::cout <<"RACK STR: " << rack_str.str() << std::endl;

                            //std::string const& xfer_back_str = "h.xfer(" + std::to_string(other_bed) + "s"+ std::to_string(stitches[s_from].bn_0.second)
                            //        + "," + std::to_string(start_bed) + std::to_string(stitches[s_to].bn_0.second) + ");";

                            std::ostringstream xfer_back_str;
                            xfer_back_str << "h.xfer('" << other_bed << 's'  << stitches[s_from].bn_0.second << "', '" << start_bed <<
                                             stitches[s_to].bn_0.second << "');";
                            std::cout <<"XFER BACK STR: " << xfer_back_str.str() << std::endl;
                            add_instr(xfer_back_str.str());

                        }

                        add_instr("h.setRacking();"); // make sure we are racked to 0

                        // everything should have come back on the original bed.


                    }
                    else{
                        std::cout << "CABLES across two beds, ignoring for now.. " << std::endl;
                        //assert(false && "TODO IMPLEMENT");
                        //   c c c
                        //  x c c

                        //
                        // drop one of the sequence on the opposite bed
                        // but this might need a local roll
                        //
                    }

                }


            }

            check_storage_layout(storages[step.out[0]], out_left, out_shape); //DEBUG


            { //no longer need to track inter layout; will replace with out[0]:
                auto f = storage_layouts.find(&step.inter);
                assert(f != storage_layouts.end());
                storage_layouts.erase(f);
            }

            //add new tube to layouts:
            auto ret = storage_layouts.insert(std::make_pair(&storages[step.out[0]], std::make_pair(out_left, out_shape)));
            assert(ret.second);

        }



        { //make sure everything gets to the final output position:
            auto old_layouts = storage_layouts;

            for (auto const &sl : step_storages[stepi]) {
                auto f = storage_layouts.find(&storages[sl.storage]);
                assert(f != storage_layouts.end());
                assert(f->second.second.pack() == storage_shapes[sl.storage]);
                f->second.first = sl.left;
            }
            // final reshape at the end of a step
            // add_instr("h.raw_inst(';reshape at the end of the step');");

            reshape(old_layouts, storage_layouts);
        }

        //Check that locations in loop-tracking arrays are as expected:
        check_storage_layouts(stepi);
    } // for-each step-i
    add_instr("h.write();");

    std::cout <<"end scheduling. Made " <<stitches.size() << " stitches." <<std::endl;

    return true;
}

bool Scheduler::write_schedule(std::string filename){

    assert(!instructions.empty());
    // write instructions to file
    std::ofstream out(filename, std::ios::binary);
    for(auto inst:instructions){
        out << inst << " \n";
    }
    out.close();
    std::cout <<"Wrote output file  " << filename << std::endl;
	return true;
}

#if 0
bool Scheduler::copy_needle_info(Polyhedron *P){
    // TODO does not display cable information currently

    for(auto &st : stitches){
        //std::cout << "st.fid = " << (int)st._fid << " "<< st.bed << st.needle << std::endl;
        if(st._fid >= 0 && st._fid < P->numFaces()){

            P->face(st._fid)->SetStep(st.step);
            P->face(st._fid)->l0 = st.bn_0;
            P->face(st._fid)->l1 = st.bn_1;

            assert(st.step >= 0);
            assert(P->face(st._fid)->l0.first != 'u');
        }
       // else{
       //     std::cout <<"stitch was not assigned to any face?! " << st.to_string() << std::endl;
       //  }
    }

    return true;
}
#endif 

void Scheduler::dump_svg(std::string filename){

    std::ofstream svg(filename);

    svg  << "<svg version='1.1' baseProfile='full' width='300' height='200'>\n";
    int min_step = std::numeric_limits<int>::infinity();
    int max_step = 0;
    int min_needle = std::numeric_limits<int>::infinity();;
    int max_needle = 0 ;
    bool made_stitches = false;

    auto get_coordinate = [](const Stitch &st)->Vector3{
        if(st.bn_0.first == 'u'){
            return Vector3(0,0,0);
        }
        bool front = (st.bn_0.first == 'f');
        int ndl = st.bn_0.second;
        float x_offset = (front ? 0: 2);
        auto p = Vector3(ndl*4 + x_offset, st.step*3 , (front ? 1: -1));
        return p;
    };

    auto DrawCircle = [&](Vector3 p, Vector3 c){
        svg << "<circle cx= '" << std::to_string(p.x()) << "' cy='"
             << std::to_string(p.y()) <<"' r= '0.25' fill='rgb("+std::to_string(c.x())+","+std::to_string(c.y())+","+std::to_string(c.z())+ ")'/>" << "\n";
    };
    auto DrawSegment = [&](Vector3 p0, Vector3 p1, Vector3 c){
       //<line x1="0" y1="0" x2="200" y2="200" style="stroke:rgb(255,0,0);stroke-width:2" />
        svg <<"<line x1='" << std::to_string(p0.x()) <<"' y1='"<<
              std::to_string(p0.y()) << "' x2='" <<
              std::to_string(p1.x()) << "' y2='" <<
              std::to_string(p1.y()) <<"' style='stroke:black;stroke-width:5'/>\n";
              //std::to_string(p1.y()) <<"' style='stroke:rgb("+std::to_string(c.x())+","+std::to_string(c.y())+","+std::to_string(c.z())+ ");stroke-width:5'/>\n";
    };
    auto DrawSegmentHorizontal = [&](Vector3 p0, Vector3 p1, Vector3 c){
        DrawSegment(p0,p1,c);
    };
    for(auto &st : stitches){


        // loop 0
        if(st.bn_0.first != 'u')
        {
            Vector3 c; c.setZero();// = colorConverter(indexcolors[st.step % 128]);
           	c.x() *= 255; c.y() *= 255; c.z() *= 255;
			auto p = get_coordinate(st);
            if(p.z() < 0){
                c.x() *= 0.6f;
                c.y() *= 0.6f;
                c.z() *= 0.6f;
            }
            DrawCircle(p,c);
            min_step = std::min(min_step, (int)p.y());
            max_step = std::max(max_step, (int)p.y());
            min_needle = std::min(min_needle, (int)p.x());
            max_needle = std::max(max_needle, (int)p.x());

        }



    }


    if(max_needle == 0) return; // nothing to draw really


    for(auto &st : stitches){

        int id = &st - &stitches[0];

        // loop 0
        if(st.bn_0.first != 'u')
        {
            Vector3 c;// = colorConverter(indexcolors[st.step % 128]);
			c.setRandom();
            auto p = get_coordinate(st);
            if(p.z() < 0){
                c.x() *= 0.6f;
                c.y() *= 0.6f;
                c.z() *= 0.6f;
            }


            if(st.in[0] != -1U){
                Vector3 n = get_coordinate(stitches[st.in[0]]);
                n = 0.5*(n+p);
                DrawSegment(n,p,c);
            }
            if(st.in[1] != -1U){
                Vector3 n = get_coordinate(stitches[st.in[1]]);
                n = 0.5*(n+p);
                DrawSegment(n,p,c);
            }


            if(st.out[0] != -1U){

                Vector3 n = get_coordinate(stitches[st.out[0]]);
                n = 0.5*(n+p);

                DrawSegment(p, n,c);
            }
            // get top right
            if(st.out[1] != -1U){

                Vector3 n = get_coordinate(stitches[st.out[1]]);
                n = 0.5*(n+p);

                DrawSegment(p, n,c);
            }
        }
        if(id > 0 && stitches[id-1].yarn == st.yarn){
        auto p0 = get_coordinate(stitches[id-1]);
        auto p1 = get_coordinate(st);
        Vector3 c ;//= colorConverter(indexcolors[st.yarn % 128]);
		c.setZero();
        //c.array() *= 255;

        if(p0.z() < 0){
            c.x() *= 0.8f;
            c.y() *= 0.8f;
            c.z() *= 0.8f;
        }
        DrawSegmentHorizontal(p0,p1,c);
        }
    }

   svg<<"</svg>\n";
   svg.close();




}

}