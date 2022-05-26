/*******************************************************************************
 * Copyright 2021 MINRES Technologies GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "axi/axi_target.h"
#include "scc/report.h"

using namespace axi;

axi_target_base::axi_target_base(const sc_core::sc_module_name& nm, axi::pe::axi_target_pe_b& pe)
: sc_module(nm)
, pe(pe) {
    SC_HAS_PROCESS(axi_target_base);
    SC_THREAD(trans_queue);
}

unsigned axi_target_base::access(tlm::tlm_generic_payload& trans) {
    peq.notify(&trans);
    return std::numeric_limits<unsigned>::max();
}

void axi_target_base::trans_queue() {
    auto delay = sc_core::SC_ZERO_TIME;
    while(true) {
        auto trans = peq.get();
        trans->acquire();
        isck->b_transport(*trans, delay);
        trans->release();
        pe.operation_resp(*trans, 0);
    }
}
