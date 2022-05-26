#include <iss/arch_if.h>
#include <iss/plugin/pctrace.h>
#include <util/logging.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/error/en.h>
#include <fstream>

#include <iostream>


using namespace rapidjson;
using namespace std;

iss::plugin::cov::cov(std::string const &filename)
    : instr_if(nullptr)
    , filename(filename)
{
    output.open("output.trc");
    jumped = false;
    first = true;
}

iss::plugin::cov::~cov() {
    output.close();
}

bool iss::plugin::cov::registration(const char *const version, vm_if& vm) {
    instr_if = vm.get_arch()->get_instrumentation_if();
    if(!instr_if) return false;
    const string  core_name = instr_if->core_type_name();
    if (filename.length() > 0) {
        ifstream is(filename);
        if (is.is_open()) {
            try {
                IStreamWrapper isw(is);
                Document d;
                ParseResult ok = d.ParseStream(isw);
                if(ok) {
                    Value& val = d[core_name.c_str()];
                    if(val.IsArray()){
                        delays.reserve(val.Size());
                        for (auto it = val.Begin(); it != val.End(); ++it) {
                            auto& name = (*it)["name"];
                            auto& size = (*it)["size"];
                            auto& delay = (*it)["delay"];
                            auto& branch = (*it)["branch"];
                            if(delay.IsArray()) {
                                auto dt = delay[0].Get<unsigned>();
                                auto dnt = delay[1].Get<unsigned>();
                                delays.push_back(instr_desc{size.Get<unsigned>(), dt, dnt, branch.Get<bool>()});
                            } else if(delay.Is<unsigned>()) {
                                auto d = delay.Get<unsigned>();
                                delays.push_back(instr_desc{size.Get<unsigned>(), d, d, branch.Get<bool>()});
                            } else
                                throw runtime_error("JSON parse error");

                        }
                    } else {
                        LOG(ERR)<<"plugin cycle_estimate: could not find an entry for "<<core_name<<" in JSON file"<<endl;
                        return false;
                   }
                } else {
                    LOG(ERR)<<"plugin cycle_estimate: could not parse in JSON file at "<< ok.Offset()<<": "<<GetParseError_En(ok.Code())<<endl;
                    return false;
               }
            } catch (runtime_error &e) {
                LOG(ERR) << "Could not parse input file " << filename << ", reason: " << e.what();
                return false;
            }
        } else {
            LOG(ERR) << "Could not open input file " << filename;
            return false;
        }
    }
    return true;



}

inline string formatPC(uint64_t pc) {
    stringstream stream;
    stream << "0x" << std::hex << pc;
    return stream.str();
}

void iss::plugin::cov::callback(instr_info_t iinfo, const exec_info& einfo) {
//    auto delay = 0;
//    auto entry = delays[iinfo.instr_id];
//    bool taken = einfo.branch_taken;
//    if (einfo.branch_taken)
//        delay = entry.taken;
//    else
//        delay = entry.not_taken;
//
//    if (first){
//        output << formatPC(instr_if->get_pc()) << "," << delay;
//        first = false;
//    }
//    if(instr_if->get_next_pc()-instr_if->get_pc() != delays[iinfo.instr_id].size/8){
//        //The goal is to keep the output in start-target pairs, so after a jump the target address needs to get written
//        //to the output. If the target happens to also be a start, we keep the pairing by adding a 0-delay entry.
//        if (jumped)
//            output <<"\n" <<formatPC(instr_if->get_pc()) << "," << 0;
//        output <<"\n" << formatPC(instr_if->get_pc()) << "," << delay;
//        jumped = true;
//    }
//    else{
//        if (jumped){
//            output <<"\n" << formatPC(instr_if->get_pc()) << "," << delay;
//            jumped = false;
//        }
//        else if(delay!=1){
//            output <<"\n" << formatPC(instr_if->get_pc()) << "," << delay;
//            output <<"\n" << formatPC(instr_if->get_pc()) << "," << 0;
//        }
//
//    }

//source code for the full output
    auto delay = 0;
    auto entry = delays[iinfo.instr_id];
    bool taken = einfo.branch_taken;
    if (einfo.branch_taken)
        delay = entry.taken;
    else
       delay = entry.not_taken;
    output<<std::hex <<"0x" << instr_if->get_pc() <<"," << delay << "\n";
}
