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

#pragma once

#include <axi/axi_tlm.h>
#include <tlm/scc/scv/tlm_rec_target_socket.h>
#include <tlm/scc/target_mixin.h>

#include <systemc>
#include <tlm>

#include <memory>
#include <queue>
#include <unordered_map>

namespace axi {

template <unsigned int BUSWIDTH = 32, unsigned int ADDRWIDTH = 32>
class axi4lite_tlm2pin_adaptor : public sc_core::sc_module {
public:
    SC_HAS_PROCESS(axi4lite_tlm2pin_adaptor);

    using payload_type = axi::axi_protocol_types::tlm_payload_type;
    using phase_type = axi::axi_protocol_types::tlm_phase_type;

    axi4lite_tlm2pin_adaptor(sc_core::sc_module_name nm);

    tlm::scc::target_mixin<tlm::scc::scv::tlm_rec_target_socket<BUSWIDTH, axi::axi_protocol_types>,
                           axi::axi_protocol_types>
        input_socket{"input_socket"};

    sc_core::sc_in<bool> clk_i{"clk_i"};
    sc_core::sc_in<bool> resetn_i{"resetn_i"};

    // Write address channel signals
    sc_core::sc_out<sc_dt::sc_uint<ADDRWIDTH>> aw_addr_o{"aw_addr_o"};
    sc_core::sc_in<bool> aw_ready_i{"aw_ready_i"};
    sc_core::sc_out<bool> aw_valid_o{"aw_valid_o"};
    sc_core::sc_out<sc_dt::sc_uint<3>> aw_prot_o{"aw_prot_o"};

    // write data channel signals
    sc_core::sc_out<sc_dt::sc_biguint<BUSWIDTH>> w_data_o{"w_data_o"};
    sc_core::sc_out<sc_dt::sc_uint<BUSWIDTH / 8>> w_strb_o{"w_strb_o"};
    sc_core::sc_out<bool> w_valid_o{"w_valid_o"};
    sc_core::sc_in<bool> w_ready_i{"w_ready_i"};

    // write response channel signals
    sc_core::sc_in<bool> b_valid_i{"b_valid_i"};
    sc_core::sc_out<bool> b_ready_o{"b_ready_o"};
    sc_core::sc_in<sc_dt::sc_uint<2>> b_resp_i{"b_resp_i"};

    // read address channel signals
    sc_core::sc_out<bool> ar_valid_o{"ar_valid_o"};
    sc_core::sc_in<bool> ar_ready_i{"ar_ready_i"};
    sc_core::sc_out<sc_dt::sc_uint<ADDRWIDTH>> ar_addr_o{"ar_addr_o"};
    sc_core::sc_out<sc_dt::sc_uint<3>> ar_prot_o{"ar_prot_o"};

    // Read data channel signals
    sc_core::sc_in<sc_dt::sc_biguint<BUSWIDTH>> r_data_i{"r_data_i"};
    sc_core::sc_in<sc_dt::sc_uint<2>> r_resp_i{"r_resp_i"};
    sc_core::sc_in<bool> r_valid_i{"r_valid_i"};
    sc_core::sc_out<bool> r_ready_o{"r_ready_o"};

    tlm::tlm_sync_enum nb_transport_fw(payload_type& trans, phase_type& phase, sc_core::sc_time& t);

private:
    /**
     * a handle class holding the pointer to a transaction payload and associated phase
     */
    struct trans_handle {
        //! pointer to the associated AXITLM payload
        payload_type* payload = nullptr;
        //! current protocol phase
        phase_type phase = tlm::UNINITIALIZED_PHASE;
        //! beat counter
        unsigned beat_cnt = 0;
    };

    std::queue<std::shared_ptr<trans_handle>> active_w_transactions;
    std::queue<std::shared_ptr<trans_handle>> active_r_transactions;
    void register_trans(payload_type& trans, phase_type& phase);

    void bus_thread();
};

/////////////////////////////////////////////////////////////////////////////////////////
// Class definition
/////////////////////////////////////////////////////////////////////////////////////////
template <unsigned int BUSWIDTH, unsigned int ADDRWIDTH>
inline axi4lite_tlm2pin_adaptor<BUSWIDTH, ADDRWIDTH>::axi4lite_tlm2pin_adaptor(sc_core::sc_module_name nm)
: sc_module(nm) {
    input_socket.register_nb_transport_fw(
        [this](payload_type& trans, phase_type& phase, sc_core::sc_time& t) -> tlm::tlm_sync_enum {
            return nb_transport_fw(trans, phase, t);
        });

    SC_METHOD(bus_thread)
    sensitive << clk_i.pos() << resetn_i.neg();
}

template <unsigned int BUSWIDTH, unsigned int ADDRWIDTH>
inline tlm::tlm_sync_enum axi4lite_tlm2pin_adaptor<BUSWIDTH, ADDRWIDTH>::nb_transport_fw(payload_type& trans,
                                                                                         phase_type& phase,
                                                                                         sc_core::sc_time& t) {
    if(trans.has_mm())
        trans.acquire();

    SCCTRACE(SCMOD) << phase << " of " << (trans.is_read() ? "RD" : "WR") << " forward trans " << std::hex << &trans;
    tlm::tlm_sync_enum status{tlm::TLM_ACCEPTED};

    if(phase == axi::BEGIN_PARTIAL_REQ || phase == tlm::BEGIN_REQ) {
        auto ext = trans.get_extension<axi::axi4_extension>();
        sc_assert(ext && "axi4_extension missing");

        if(trans.is_read()) {
            register_trans(trans, phase);
        } else { // Write transaction
            if(active_w_transactions.empty())
                register_trans(trans, phase);
            else {
                auto act_trans = active_w_transactions.front();
                if(act_trans->payload == &trans) // if transaction is ongoing update the phase
                    act_trans->phase = phase;
                else // otherwise push transaction in the queue
                    register_trans(trans, phase);
            }
        }
    } else if(phase == tlm::END_RESP) {
        trans.set_response_status(tlm::TLM_OK_RESPONSE);
        if(trans.is_read())
            active_r_transactions.pop();
        else
            active_w_transactions.pop();

        if(trans.has_mm())
            trans.release();
        status = tlm::TLM_ACCEPTED;
    }
    return status;
}

template <unsigned int BUSWIDTH, unsigned int ADDRWIDTH>
inline void axi4lite_tlm2pin_adaptor<BUSWIDTH, ADDRWIDTH>::bus_thread() {
    auto delay = sc_core::SC_ZERO_TIME;

    if(!resetn_i.read()) { // active-low reset
        ar_valid_o.write(false);
        aw_valid_o.write(false);
        w_valid_o.write(false);
        b_ready_o.write(false);
        r_ready_o.write(false);
    } else {
        if(aw_ready_i.read())
            aw_valid_o.write(false);
        if(w_ready_i.read()) {
            w_valid_o.write(false);
        }
        if(ar_ready_i.read()) {
            ar_valid_o.write(false);
        }
        r_ready_o.write(false);
        b_ready_o.write(false);

        // AR channel
        if(!active_r_transactions.empty()) {
            auto read_trans = active_r_transactions.front();
            if(read_trans->phase == tlm::BEGIN_REQ) {
                payload_type* p = read_trans->payload;
                auto ext = p->get_extension<axi::axi4_extension>();
                sc_assert(ext && "axi4_extension missing");

                ar_addr_o.write(p->get_address());
                ar_prot_o.write(ext->get_prot());
                ar_valid_o.write(true);

                read_trans->phase = tlm::END_REQ;
                SCCTRACE(SCMOD) << read_trans->phase << " RD BW trans: " << p << " addr: " << p->get_address();
                input_socket->nb_transport_bw(*read_trans->payload, read_trans->phase, delay);
            }
        }

        // AW+W channel
        if(!active_w_transactions.empty()) {
            auto write_trans = active_w_transactions.front();
            if(write_trans->phase == axi::BEGIN_PARTIAL_REQ || write_trans->phase == tlm::BEGIN_REQ) {
                payload_type* p = write_trans->payload;
                auto ext = p->get_extension<axi::axi4_extension>();
                sc_assert(ext && "axi4_extension missing");

                if(write_trans->beat_cnt == 0) { // TODO: check if required for axi lite
                    aw_addr_o.write(p->get_address());
                    aw_prot_o.write(ext->get_prot());
                    aw_valid_o.write(true);
                }
                write_trans->beat_cnt++;

                // write data
                sc_dt::sc_biguint<BUSWIDTH> data{0};
                for(size_t j = 0, k = 0, offset = 0; k < 32 / 8; j += 8, ++k, ++offset)
                    data.range(j + 7, j) = *(uint8_t*)(p->get_data_ptr() + offset);

                w_strb_o.write(p->get_byte_enable_length());
                w_data_o.write(data);
                w_valid_o.write(true);

                write_trans->phase =
                    (write_trans->phase == axi::BEGIN_PARTIAL_REQ) ? axi::END_PARTIAL_REQ : tlm::END_REQ;
                SCCTRACE(SCMOD) << write_trans->phase << " RD BW trans: " << p << " addr: " << p->get_address();
                input_socket->nb_transport_bw(*write_trans->payload, write_trans->phase, delay);
            }
        }

        // The Slave puts the requested data on the Read Data channel and asserts RVALID,
        // indicating the data in the channel is valid.
        if(r_valid_i.read()) {
            if(!active_r_transactions.empty()) {
                auto read_trans = active_r_transactions.front();
                if(read_trans->payload == nullptr)
                    SCCERR(SCMOD) << "Invalid transaction";

                if(read_trans->phase == tlm::END_REQ) {
                    auto data = r_data_i.read();
                    for(size_t j = 0, k = 0; k < BUSWIDTH / 8; j += 8, ++k) {
                        *(uint8_t*)(read_trans->payload->get_data_ptr() + k) = data.range(j + 7, j).to_uint();
                    }

                    // The Master asserts RREADY, indicating the master is ready to receive data from the slave.
                    r_ready_o.write(true);
                    read_trans->phase = tlm::BEGIN_RESP;

                    auto ret = input_socket->nb_transport_bw(*read_trans->payload, read_trans->phase, delay);
                    SCCTRACE(SCMOD) << read_trans->phase << " bw trans " << std::hex << read_trans->payload;

                    // EDN_RESP indicates the last phase of the AXI Read transaction
                    if(ret == tlm::TLM_UPDATED && read_trans->phase == tlm::END_RESP) {
                        active_r_transactions.pop();
                        if(read_trans->payload->has_mm())
                            read_trans->payload->release();
                    }
                }
            }
        }

        if(b_valid_i.read()) {
            if(!active_w_transactions.empty()) {
                auto write_trans = active_w_transactions.front();
                if(write_trans->payload == nullptr)
                    SCCERR(SCMOD) << "Invalid transaction";

                if(write_trans->phase == tlm::END_REQ) {
                    sc_assert(b_resp_i.read() == axi::to_int(axi::resp_e::OKAY));
                    b_ready_o.write(true);
                    write_trans->phase = tlm::BEGIN_RESP;
                    auto ret = input_socket->nb_transport_bw(*write_trans->payload, write_trans->phase, delay);
                    SCCTRACE(SCMOD) << write_trans->phase << " bw trans " << std::hex << write_trans->payload;
                    if(ret == tlm::TLM_UPDATED && write_trans->phase == tlm::END_RESP) {
                        active_w_transactions.pop();
                    }
                }
            }
        }
    }
}

template <unsigned int BUSWIDTH, unsigned int ADDRWIDTH>
void axi4lite_tlm2pin_adaptor<BUSWIDTH, ADDRWIDTH>::register_trans(payload_type& trans, phase_type& phase) {
    auto th = std::make_shared<trans_handle>();
    th->payload = &trans;
    th->phase = phase;
    if(trans.is_read())
        active_r_transactions.push(th);
    else
        active_w_transactions.push(th);
}

} // namespace axi
