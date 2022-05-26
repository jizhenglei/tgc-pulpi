/*******************************************************************************
 * Copyright (C) 2017, MINRES Technologies GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Contributors:
 *       eyck@minres.com - initial API and implementation
 ******************************************************************************/

#include "iss/plugin/cycle_estimate.h"

#include <iss/arch_if.h>
#include <util/logging.h>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/error/en.h>
#include <fstream>

using namespace rapidjson;
using namespace std;

iss::plugin::cycle_estimate::cycle_estimate(string const& config_file_name)
: arch_instr(nullptr)
, config_file_name(config_file_name)
{
}

iss::plugin::cycle_estimate::~cycle_estimate() {
}

bool iss::plugin::cycle_estimate::registration(const char* const version, vm_if& vm) {
    arch_instr = vm.get_arch()->get_instrumentation_if();
    if(!arch_instr) return false;
    const string  core_name = arch_instr->core_type_name();
    if (config_file_name.length() > 0) {
        ifstream is(config_file_name);
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
                LOG(ERR) << "Could not parse input file " << config_file_name << ", reason: " << e.what();
                return false;
            }
        } else {
            LOG(ERR) << "Could not open input file " << config_file_name;
            return false;
        }
    }
    return true;

}

void iss::plugin::cycle_estimate::callback(instr_info_t instr_info, exec_info const& exc_info) {
    assert(arch_instr && "No instrumentation interface available but callback executed");
    auto entry = delays[instr_info.instr_id];
    bool taken = exc_info.branch_taken;
    if (exc_info.branch_taken && (entry.taken > 1))
        arch_instr->set_curr_instr_cycles(entry.taken);
    else if (entry.not_taken > 1)
        arch_instr->set_curr_instr_cycles(entry.not_taken);
}
