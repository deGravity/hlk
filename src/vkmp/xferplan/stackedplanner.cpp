#include <cstdint>
#include "stackedplanner.h"
#include <iostream>
#include <set>
#include <unordered_set>
#include <queue>

namespace vkmp {

bool stacked_planner(Constraints const &constraints,
                     std::vector<BedNeedle> from,
                     std::vector<BedNeedle> to,
                     std::vector<int> firsts,
                     std::vector<Slack> slack, // slack with next
                     std::vector<Transfer> *transfers_
                     ){

    // ----------------setup from plan transfers------------
    assert(constraints.min_free < constraints.max_free);
    assert(constraints.max_racking >= 1);

    assert(from.size() == to.size());
    assert(from.size() == slack.size());
    assert(from.size() == firsts.size());
    assert(transfers_);

    auto &transfers = *transfers_;
    transfers.clear();

    if (from.empty()) return true; //empty cycle has empty plan


    //eliminate any stacked needles in 'from':
    for (uint32_t i = 0; i < from.size(); /* later */) {
        if (from.size() == 1) break;
        uint32_t n = (i + 1 < from.size() ? i + 1 : 0);
        if (from[i] == from[n]) {
            assert(to[i] == to[n]);
            from.erase(from.begin() + i);
            to.erase(to.begin() + i);
            slack.erase(slack.begin() + i);
            firsts.erase(slack.begin()+i);
        } else {
            ++i;
        }
    }
    const uint32_t Count = from.size();
    assert(from.size() == Count);
    assert(to.size() == Count);
    assert(slack.size() == Count);
    assert(firsts.size() == Count);

    //PARANOIA: we did actually eliminate duplicates, right?
    if (from.size() > 1) {
        for (uint32_t i = 0; i < from.size(); ++i) {
            uint32_t n = (i + 1 < Count ? i + 1 : 0);
            assert(!(from[i] == from[n]));
        }
    }

    //-------------end of setup from plan transfers-------------

    std::vector<Transfer> cse_xfers;
    // if there are no firsts, just use collapse stretch expand and return
    {
        std::string cse_err;
        bool planned = plan_transfers(constraints,from,to,slack, &cse_xfers, &cse_err);
        if(!planned){
            std::cerr << "CSE failed "<< cse_err << std::endl;
        }
        assert(planned);

    }

    //-------------we really need the stacked planner -----------

    auto transfer_front = [](const Transfer& t)->int{
        if(t.from.bed == BedNeedle::Front || t.from.bed == BedNeedle::FrontSliders){
            return t.from.needle;
        }
        else{
            assert(t.to.bed == BedNeedle::Front || t.to.bed == BedNeedle::FrontSliders);
            return t.to.needle;
        }
    };
    auto transfer_back = [](const Transfer& t)->int{
        if(t.from.bed == BedNeedle::Back || t.from.bed == BedNeedle::BackSliders){
            return t.from.needle;
        }
        else{
            assert(t.to.bed == BedNeedle::Back || t.to.bed == BedNeedle::BackSliders);
            return t.to.needle;
        }
    };
    struct State{
        std::vector<BedNeedle> currents; // from
        int penalty = 0;
        int passes = 0;
        int rack = 0;
        int cse_estimate = 0;
        std::vector<Transfer> xfers; // xfers accrued so far
    };
    auto front_or_frontslider = [=](const BedNeedle& bn)->bool{
      if(bn.bed == BedNeedle::Front || bn.bed == BedNeedle::FrontSliders) return true;
      return false;
    };

    auto back_or_backslider = [=](const BedNeedle& bn)->bool{
      if(bn.bed == BedNeedle::Back || bn.bed == BedNeedle::BackSliders) return true;
      return false;
    };

    auto apply_transfer_to_state = [=](const State& s, Transfer x)->State{
      State n = s;
      for(auto &bn : n.currents){
          if(bn == x.from){
              bn = x.to;
          }
      }
      n.xfers.push_back(x);
      n.rack = transfer_front(x) - transfer_back(x);
      return n;
    };

    auto Penalty = [=](const State& s)->int{
      int p = 0;
      for(int i = 0; i < s.currents.size(); i++){

          p += std::abs(s.currents[i].needle - to[i].needle);
          if(front_or_frontslider( s.currents[i] )!= front_or_frontslider(to[i])){
               int f = front_or_frontslider(s.currents[i]) ? s.currents[i].needle : to[i].needle;
               int b = back_or_backslider(s.currents[i]) ? s.currents[i].needle : to[i].needle;
               int p1 = (2*constraints.max_free - (b+f));
               int p2 = (b+f - 2*constraints.min_free);
               p += std::min(p1,p2);

          }
          if(firsts[i]) // wherever stitch i want's to go, it wants to reach there first
          {
              // as long as currents[i] is not already stacked
              // or whatever is stacked on it didn't reached it first
              // we shouldn't penalize
              // will need to go through history here
          }
      }
      return p;
    };
    auto Passes = [=](const std::vector<Transfer> xfers, int initial_rack = 0)->int{
        int r = initial_rack;
        int p = 0;
        for(auto const &t : xfers){
            int f = transfer_front(t);
            int b = transfer_back(t);
            int rr = f-b;
            if(rr != r){
                r = rr;
                p++;
            }
        }
        return p; //no. of passes the cse transfer planner used
    };
    auto Offset_Bound = [=](const State &s)->int{
      std::unordered_set<int> required_racks;
      for(int i = 0; i < s.currents.size(); i++){
        required_racks.insert(s.currents[i].needle - to[i].needle);
      }
      required_racks.erase(0);
      return required_racks.size();

    };
    auto CSE_Bound = [=](const State &s)->int{
        return Offset_Bound(s);
        // Unfortunatley most states wont be in a valid front bed
        // back bed configuration to starrt cse from
        /*
        std::vector<Transfer> xfers;
        std::string error_str;
        bool planned = plan_transfers(constraints,s.currents,to,slack, &xfers, &error_str);
        if(!planned){
            std::cerr << "CSE failed "<< error_str << std::endl;
        }
        assert(planned);
        // compute  bound from collapse-stretch-expand
        return Passes(xfers, s.rack);
        */
    };

    auto opposite = [=](const BedNeedle& bn)->BedNeedle{

        if(bn.bed == BedNeedle::Front) return BedNeedle(BedNeedle::BackSliders,bn.needle);
      else if(bn.bed == BedNeedle::Back) return  BedNeedle(BedNeedle::FrontSliders,bn.needle);
      else if(bn.bed == BedNeedle::FrontSliders) return BedNeedle(BedNeedle::Back,bn.needle);
      else if(bn.bed == BedNeedle::BackSliders) return  BedNeedle(BedNeedle::Front,bn.needle);
      assert(false && "invalid bed type.");

      return BedNeedle(BedNeedle::Front,0);
    };



    auto is_state_valid = [=](const State &s, bool verbose = false)->bool{

        // State does not violate constraints:
        for(int i = 0; i < s.currents.size(); i++){
            if(s.currents[i].needle < constraints.min_free ||
                    s.currents[i].needle > constraints.max_free){
                if(verbose){
                    std::cout<<"[Min max free range] violated by index " << i << " i.e " << s.currents[i].to_string()
                            <<" Constraints: " << constraints.min_free <<","<< constraints.max_free << std::endl;
                }
                return false;
            }
        }
        // State does not cable:
        std::vector< std::pair<BedNeedle, BedNeedle> > bridges;
        for(int i = 1; i < s.currents.size(); i++){
            auto c = s.currents[i];
            auto n = s.currents[(i+1)%s.currents.size()];
            if((front_or_frontslider(c) && back_or_backslider(n))
               ||(back_or_backslider(c) && front_or_frontslider(n))
                    ){
                if(c.bed == BedNeedle::Front || c.bed == BedNeedle::FrontSliders)
                    bridges.push_back(std::make_pair(c,n));
                else
                    bridges.push_back(std::make_pair(n,c));
            }
        }
        for(int i = 0; i < bridges.size(); i++){
            for(int j = i+1; j < bridges.size(); j++){
                // if bridge i and j are not valid, return false
                if( (bridges[i].first.needle <= bridges[j].first.needle &&
                     bridges[i].second.needle <= bridges[j].second.needle) ||
                        (bridges[i].first.needle >= bridges[j].first.needle &&
                         bridges[i].second.needle >= bridges[j].second.needle) ){
                    //okay
                }
                else{
                    if(verbose){
                    std::cout << "Bridge violated " << bridges[i].first.to_string() << "," << bridges[i].second.to_string() <<
                                 " and " << bridges[j].first.to_string() <<", "<< bridges[j].second.to_string() << std::endl;
                    }
                        return false;
                }
            }
        }
        // No operation stacks more than 2!, only need to check the last
        if(s.xfers.size())
        {
            auto last = s.xfers.back().to;
            int c = 0;
            for(auto bn : s.currents){
                if(last == bn){
                    c++;
                }
            }
            // there is surely one, there can't be more than one
            assert(c >= 1);
            if( c > 2) {
                if(verbose){
                std::cout <<"Stacked twice!" << std::endl;
                }
                return false; // cannot stack more than 2!
            }
        }
        // State does not violate slack:
        for(int i = 1; i < s.currents.size(); i++){
            auto c = s.currents[i];
            auto n = s.currents[(i+1)%s.currents.size()];
            if(c.bed == BedNeedle::Back || c.bed == BedNeedle::BackSliders){
                c.needle += s.rack;
            }
            if(n.bed == BedNeedle::Back || n.bed == BedNeedle::BackSliders){
                n.needle += s.rack;
            }

            int sl = std::abs(n.needle - c.needle);
            if(slack[i] < sl){
               if(verbose){
                std::cout <<"slack violation at " << i << " " << sl << " > " << slack[i] << " rack = " << s.rack << std::endl;
               }
                return false;
            }
        }

      return true;
    };
    auto can_rack = [=](const State &s, int r)->bool{
        // given state s at a racking s.r, can it be racked to r ?
        // rack = front - back
        // if front =  2, back = 3, rack = -1
        // if front =  2, back = 1, rack = +1
        // find bridges, is slack respected at a bridge when racked
        State n = s;
        n.rack = r;
        return is_state_valid(s);

    };
    auto can_move = [=](const State &s, int idx)->bool{
      // given state s at a racking s.r, can s.currents[idx] be moved to
      // its opposite bed / holding while respecting
        auto from = s.currents[idx];
        auto to = opposite(from);
        if(from.bed == BedNeedle::Front || from.bed == BedNeedle::FrontSliders){
            to.needle -= s.rack;
        }
        else{
            to.needle += s.rack;
        }
        State n = s;

        if(firsts[idx]){
            for(auto x : s.currents){
                if(x == to){
                    // there is already somebody where idx wants to go
                    // and idx wanted to reach there first
                    return false;
                }
            }
        }
        auto f = n.currents[idx];
        for(auto &x : n.currents){
            if(x == f){
                x = to;
            }
        }
        n.currents[idx] = to;
        n.xfers.push_back(Transfer(from,to));
        return is_state_valid(n);

    };
    struct LessThanForState{
        bool operator()(const State& lhs, const State& rhs) const{
//            return (  lhs.cse_estimate + lhs.penalty > rhs.cse_estimate + rhs.penalty);
            return ( ( lhs.passes + lhs.cse_estimate + lhs.penalty)
                     > (rhs.passes + rhs.cse_estimate + rhs.penalty));

        }
    };

    auto print_state = [=](State const s){
      std::cout<<"State racking = " << s.rack <<"  [";
      for(auto bn: s.currents){
          std::cout << (char)bn.bed << bn.needle <<",";
      }
      std::cout <<"]. Penalty = "
               << s.penalty <<" Passes = "
               << s.passes << " #xfers "
               << s.xfers.size() <<". "<<std::endl;
    };
    std::priority_queue<State, std::vector<State>, LessThanForState > PQ;
    State start;
    start.currents = from;
    start.penalty = Penalty(start);
    start.cse_estimate = CSE_Bound(start);
    PQ.push(start);

    //for all cse xfers that don't violate state, apply and generate a starting state
    for(auto x : cse_xfers)
    {
        State next = apply_transfer_to_state(start, x);
        if(is_state_valid(next)){
            next.passes = Passes(next.xfers);
            next.penalty = Penalty(next); //fake penalty so that we begin exploring these
            next.cse_estimate = CSE_Bound(next);
            PQ.push(next);
            start = next;
            std::cout << "Penalty = " << next.penalty
                      <<" Passes = " << next.passes
                     << " Est = " << next.cse_estimate
                     << std::endl;
        }
        else{
            std::cout<<"Applying " << x.from.to_string() <<"->"<<x.to.to_string() <<"failed?" <<std::endl;
            std::cout<<"State is valid ? \n" << is_state_valid(next, true) << std::endl;
            print_state(next);
            break;
        }
    }

    std::cout <<"Cse xfers = " << cse_xfers.size() << " enqueued " << PQ.size()-1 << " of them."<<std::endl;

    std::set< std::pair<int, std::vector<BedNeedle> > > visited;
    //std::unordered_set< std::pair<int, std::vector<BedNeedle>> > vv;
    std::cout <<"Constraints = ["<<constraints.min_free<<","<<constraints.max_free<<"] max rack "<<constraints.max_racking << std::endl;
    std::cout <<"Stacked Planning: From :";
    for(auto t : from){
        std::cout<<(char)t.bed << t.needle <<",";
    }
    std::cout<<std::endl<<"Stacked Planning: To  :";
    for(auto t : to){
        std::cout<<(char)t.bed << t.needle <<",";
    }

    std::cout << std::endl;
    while(!PQ.empty()){
        auto s = PQ.top();
        PQ.pop();
        //print_state(s);
        if(Penalty(s) == 0){
            // Reached state! Might not be optimal since this is not closed
            // and also not an admissible heuristic
            // But... Reached state!
            transfers = s.xfers;
            // TODO check that this is indeed correct
            return true;
        }

        // Are we revisiting an old state i.e same currents, same rack ?
        // #xfers is immaterial if we reached the same spot
        auto key = std::make_pair(s.rack, s.currents);

        if(visited.count(key))
        {
            //std::cout <<"visited." << std::endl;
            continue; // ideally we should be depth first making progress no ?
        }

        visited.insert(std::make_pair(s.rack,s.currents));

        // At this state the following can be done:
        // Rack to some variation
        int rack_states = 0;
        int max_rack = constraints.max_racking;
        for(int r = -max_rack ; r <= max_rack; r++){
            if(s.rack != r && can_rack(s,r)){
                State t = s;
                t.rack = r;
                t.cse_estimate = CSE_Bound(t);
                t.penalty = s.penalty;
                t.passes = s.passes; // penalty and passes don't change
                if(!visited.count(std::make_pair(t.rack,t.currents)))
                {
                    PQ.push(t);
                    rack_states++;
                }

            }
        }
        int move_states = 0;
        // Move a stitch from its current bed to the opposite bed
        for(int i = 0; i < to.size(); i++){
            if(can_move(s,i)){
                State t = s;
                Transfer x;
                // make the xfer
                x.from = t.currents[i];
                x.to = opposite(x.from);
                if(x.from.bed == BedNeedle::Front || x.from.bed == BedNeedle::FrontSliders){
                    x.to.needle -= t.rack;
                }
                else{
                    x.to.needle += t.rack;
                }
                t.currents[i] = x.to; // update current state

                // everybody that pointed to x.from should also point to i
                for(auto &e : t.currents){
                    if(e == x.from) e = x.to;
                }

                t.xfers.push_back(x); // update current xfers
                t.passes = Passes(t.xfers);
                t.cse_estimate = CSE_Bound(t);
                t.penalty = Penalty(t);
                if(!visited.count(std::make_pair(t.rack,t.currents)))
                {
                    PQ.push(t);
                    move_states++;
                }
            }
        }

        //std::cout <<"\tEnqueued rack-states: " << rack_states << " move-states: " << move_states << std::endl;

    } //while

    assert(false && "could not find any plan, perhaps fallback on cse?");

    return false;
}

}