#include <cstdint>
#include "Stitch.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>


namespace vkmp {

// TODO needs to load stitch data as well..
bool load_stitches(std::string const &filename, std::vector< Stitch > *into_) {
	assert(into_);
	auto &into = *into_;
	into.clear();

	std::ifstream file(filename);
	std::string line;
	while (std::getline(file, line)) {
		std::istringstream iss(line);
		Stitch temp;

		int32_t in[2];
		int32_t out[2];

		if (!(iss >> temp.yarn >> temp.type >> temp.direction >> in[0] >> in[1] >> out[0] >> out[1] >> temp.at.x >> temp.at.y >> temp.at.z >> temp.data.id >> temp.data.name >> temp.data.is_basic_type >>  temp.data.is_custom_xfer)) {
			std::cerr << "ERROR: Failed to read stitch: " << line << std::endl;
			return false;
		}
		temp.in[0] = in[0];
		temp.in[1] = in[1];
		temp.out[0] = out[0];
		temp.out[1] = out[1];

		if (!temp.check_type()) {
			std::cerr << "ERROR: Stitch does not have proper in/out for type." << std::endl;
			std::cerr << "  line: '" << line << "'" << std::endl;
			return false;
		}


		into.emplace_back(temp);
	}
	for (auto const &s : into) {
		uint32_t idx = &s - &into[0];
		auto check_in = [&](uint32_t in_idx) -> bool {
			if (in_idx == -1U) {
				return true;
			} else {
				if (in_idx >= idx) return false;
				if (into[in_idx].find_out(idx) == -1U) return false;
				return true;
			}
		};
		if (!check_in(s.in[0]) || !check_in(s.in[1])) {
			std::cerr << "Stitch does not have proper 'in' array." << std::endl;
			return false;
		}
		auto check_out = [&](uint32_t out_idx) -> bool {
			if (out_idx == -1U) {
				return true;
			} else {
				if (out_idx >= into.size()) return false;
				if (out_idx <= idx) return false;
				if (into[out_idx].find_in(idx) == -1U) return false;
				return true;
			}
		};
		if (!check_out(s.out[0]) || !check_out(s.out[1])) {
			std::cerr << "Stitch does not have proper 'out' array." << std::endl;
			return false;
		}
	}
	return true;
}
// TODO does not write cable information, knit purl type information and all that...
void save_stitches(std::string const &filename, std::vector< Stitch > const &from) {
	std::ofstream file(filename);
	for (auto const &s : from) {
		file << s.yarn
		<< ' ' << s.type
		<< ' ' << s.direction
		<< ' ' << (int32_t)s.in[0]
		<< ' ' << (int32_t)s.in[1]
		<< ' ' << (int32_t)s.out[0]
		<< ' ' << (int32_t)s.out[1]
		<< ' ' << s.at.x << ' ' << s.at.y << ' ' << s.at.z 
		<< ' ' << s.data.id << ' ' << s.data.name << ' ' 
		<< (int32_t) s.data.is_basic_type << ' ' << (int32_t)s.data.is_custom_xfer <<  '\n';
	}
}

void save_stitches_as_CW(std::string const &filename, std::vector< Stitch > const &from) {

    std::ofstream file(filename);
    for (auto const &s : from) {
        file
        << "s " << s.at.x << ' ' << s.at.y << ' ' << s.at.z
        << ' ' << s.type << ' ' << s.yarn << " 0 0 0"
        << '\n';

       // std::cout
       // << "s " << s.at.x << ' ' << s.at.y << ' ' << s.at.z
       // << ' ' << s.type << ' ' << "0 0 0 0 "
       // << '\n';
    }
    int max_yarns = 0;
    for (auto const &s : from) {
        int id = &s - &from[0];
        for(int i = 0; i < 2; i++){
            if(s.in[i] != -1){
                file << "w " << s.in[i] << ' ' << id << '\n';
                //std::cout <<"w " << s.in[i] << ' ' << id << '\n';

            }
            if(s.out[i] != -1){
                file << "w " << id  << ' ' << s.out[i] << '\n';
                // std::cout << "w " << id  << ' ' << s.out[i] << '\n';

            }
        }
        max_yarns = std::max(max_yarns, (int)s.yarn);

    }
    assert(max_yarns >= 0);
    std::vector< std::vector<int> > yarns;
    yarns.assign(max_yarns+1, std::vector<int>());
    for (auto const &s : from) {
        yarns[s.yarn].push_back(&s-&from[0]);
    }
    for(auto y : yarns){
        for(int i  = 1; i < y.size(); i++){
            file << "c " << y[i-1]  << ' ' << y[i] << '\n';
            //std::cout << "c " << y[i-1]  << ' ' << y[i] << '\n';

        }
    }
    // std::cout<<"num yarns = " << yarns.size() << " first yarn = " << yarns[0].size() << std::endl;
    file.close();
}

#if 0
void save_stitches_as_SM(std::string const &filename, std::vector<Stitch> const &from){
    //std::cout << "TODO save as SM, saving as CW " << std::endl;
    save_stitches_as_CW(filename, from);
    std::vector<bool> dirs; // direction of the yarn to the next(right) of this stitch
    std::vector<int> prevs, nexts, in_lefts, in_rights, out_lefts, out_rights;
    Polyhedron sm_in, sm;
    std::ofstream obj("/Users/nvidya/Desktop/temp.obj");
    for(auto s : from){
        sm_in.addVertex(s.at.x, s.at.y, s.at.z);
        obj << "v " << s.at.x << " " << s.at.y << " " << s.at.z << " \n";
    }
    dirs.assign(from.size(), false);
    prevs.assign(from.size(),-1);
    nexts.assign(from.size(),-1);
    in_lefts.assign(from.size(),-1);
    in_rights.assign(from.size(),-1);
    out_lefts.assign(from.size(),-1);
    out_rights.assign(from.size(),-1);
    bool d = false;
    std::cout << "Starting direction " << from[0].direction << std::endl;
    for(auto &s : from){
        int id = &s - &from[0];

        int p = id > 0 ? id-1 : -1;
        int n = (id+2 < from.size() ? id+1 : -1);
        if(from[n].yarn != s.yarn) n = -1;
        if(from[p].yarn != s.yarn) p = -1;


        prevs[id] = p;
        nexts[id] = n;
        in_lefts[id] = s.in[0];
        in_rights[id] = s.in[1];
        out_lefts[id] = s.out[0];
        out_rights[id] = s.out[1];

        if(s.type == Stitch::Tuck){
            std::cout << "Tuck dir " << s.direction << std::endl;
            prevs[id] = -1;
        }
        if(s.type == Stitch::Miss){
            std::cout << "Miss dir " << s.direction << std::endl;
            nexts[id] = -1;
        }
        dirs[id] = (s.direction == Stitch::Clockwise ? true : false);
    }
    for(int i = 0; i < from.size(); i++){
        std::vector<int> flist;
        flist.push_back(i);
        bool mismatch = false;
        if(nexts[i] >= 0 ){
            int n = nexts[i]; // (!dirs[i] ? nexts[i] : prevs[i]);
            flist.push_back(n);
            int u = out_lefts[n];
            if(u >= 0){
                flist.push_back(u);
                int up = prevs[u];//(dirs[n] == dirs[u]) ? prevs[u] : nexts[u];

                if(dirs[n] != dirs[u]){
                    std::cout << "Direction of up and down different. ";
                    std::cout << "Stitches " << from[n].type << " " << from[u].type << std::endl;
                    up = nexts[u];
                    mismatch = true;
                }
                else{

                }
                if(up >=0){
                    flist.push_back(up);
                    if(in_rights[up] >= 0 && in_rights[up] == i){
                        //if(flist.size() == 4)
                        if(from[flist[0]].direction == Stitch::Clockwise) std::reverse(flist.begin(), flist.end());
                        sm_in.addFace(flist);

                        obj << "f ";
                        for(auto x : flist)  obj << x+1 << " ";
                        obj << "\n";

                        std::cout << "\t added face of size " << flist.size()  << "mismatch? " << mismatch << std::endl;
                    }
                    else if(in_lefts[up] >= 0 && in_lefts[up] == i){
                        //if(flist.size()==4)
                        if(from[flist[0]].direction == Stitch::Clockwise) std::reverse(flist.begin(), flist.end());
                        sm_in.addFace(flist);

                        obj << "f ";
                        for(auto x : flist)  obj << x+1 << " ";
                        obj << "\n";

                        std::cout << "\t added face(2) of size " << flist.size() << " mismatch? " << mismatch << std::endl;

                    }
                    else{
                        std::cout << "Mismatch? " << mismatch << " . Invalid face. found [";
                        for(auto x : flist) std::cout << x << "(" << from[x].direction << ") ";
                        std::cout << "]  in l " << in_lefts[up] << " in r " << in_rights[up] ;
                        std::cout << std::endl;
                        for(auto x : flist) {
                            auto st = from[x];
                            std::cout << st.to_string() << std::endl;
                        }
                    }
                }
            }

        }
    }

    for(int i = 0; i < from.size(); i++){
        if(from[i].type == Stitch::Miss){
            std::vector<int> flist, flist2;
            int t = out_lefts[i];
            assert( from[t].type == Stitch::Tuck);
            int dn = in_lefts[i];
            assert(dn >= 0);
            int p  = prevs[dn];
            if(dirs[i] == dirs[dn]){
                p = nexts[dn];
                assert(p>=0);
                flist.push_back(p);
                flist.push_back(dn);
                flist.push_back(i);
                sm_in.addFace(flist);

                flist2.push_back(p);
                flist2.push_back(i);
                flist2.push_back(t);

                sm_in.addFace(flist2);

                obj << "f ";
                for(auto x : flist)  obj << x+1 << " ";
                obj << "\n";


                obj << "f ";
                for(auto x : flist2)  obj << x+1 << " ";
                obj << "\n";

            }
            else{
                p = prevs[dn];
                assert(p >= 0);
                flist.push_back(dn);
                flist.push_back(p);
                flist.push_back(i);
                sm_in.addFace(flist);

                flist2.push_back(i);
                flist2.push_back(p);
                flist2.push_back(t);
                sm_in.addFace(flist2);

                obj << "f ";
                for(auto x : flist)  obj << x+1 << " ";
                obj << "\n";

                obj << "f ";
                for(auto x : flist2)  obj << x+1 << " ";
                obj << "\n";

            }
        }
    }

    for(int i = 0; i < from.size(); i++){
        std::vector<int> flist;
        flist.push_back(i);
        if(out_lefts[i] >= 0 && out_rights[i] >=0 ){
            flist.push_back(out_lefts[i]);
            flist.push_back(out_rights[i]);

            auto f = sm_in.addFace(flist);

            obj << "f ";
            for(auto x : flist)  obj << x+1 << " ";
            obj << "\n";
        }

        if(in_lefts[i] >= 0 && in_rights[i] >=0 ){
            flist.push_back(in_rights[i]);
            flist.push_back(in_lefts[i]);
            auto f = sm_in.addFace(flist);
            obj << "f ";
            for(auto x : flist)  obj << x+1 << " ";
            obj << "\n";

        }
    }

    obj.close();
    std::cout << "Num verts = " << sm_in.numVertices() << " Faces " << sm_in.numFaces() << std::endl;
    sm_in.finalize();
/*
    for(int i = 0; i < sm_in.numFaces(); i++){
        auto f = sm_in.face(i);
        auto it = f->begin();
        auto sen = it;
        bool terminal = false;
        do{
            int v = (*it)->dst()->index();
            if(from[v].type == Stitch::Start || from[v].type == Stitch::End){
                terminal = true;
            }
            ++it;
        }while(it != sen);
        if(f->hole() && !terminal ){
            f->hole(false);

        }
    }*/
    std::cout << "Finalized. "<<std::endl;



    Polyhedron sm_out;
    for(int i  =0 ;i < sm_in.numFaces(); i++){
        auto f = sm_in.face(i);
        {
            sm_out.addVertex(f->centroid().x(), f->centroid().y(), f->centroid().z());
        }
    }
    for(int i = 0; i < sm_in.numVertices(); i++){
        auto v = sm_in.vertex(i);
        if(!v) continue;
        auto it = v->begin();
        auto sen = it;
        if(!(*it)) continue;
        std::vector<int> list;
        bool skip = false;
        do{

            auto f = (*it)->face();
            if(f)
                list.push_back(f->index());
            else
                skip = true;
            ++it;
        }while(it != sen);
        if(!skip)
        sm_out.addFace(list);
    }

    sm_out.finalize();

    sm_in.saveAsObj("/Users/nvidya/temp_sm_in.obj");
    sm_out.saveAsObj("/Users/nvidya/temp_sm_out.obj");


}
#endif

}