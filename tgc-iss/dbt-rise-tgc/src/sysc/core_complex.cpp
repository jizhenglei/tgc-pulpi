/*******************************************************************************
 * Copyright (C) 2017, 2018 MINRES Technologies GmbH
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
 *******************************************************************************/

// clang-format off
#include <iss/debugger/gdb_session.h>
#include <iss/debugger/encoderdecoder.h>
#include <iss/debugger/server.h>
#include <iss/debugger/target_adapter_if.h>
#include <iss/iss.h>
#include <iss/vm_types.h>
#include <iss/plugin/loader.h>
#include <sysc/core_complex.h>
#include <iss/arch/tgc_mapper.h>
#include <scc/report.h>
#include <util/ities.h>
#include <iostream>
#include <sstream>
#include <array>
#include <iss/plugin/cycle_estimate.h>
#include <iss/plugin/instruction_count.h>
#include <iss/plugin/pctrace.h>

// clang-format on

#define STR(X) #X
#define CREATE_CORE(CN) \
if (type == STR(CN)) { std::tie(cpu, vm) = create_core<CN ## _plat_type>(backend, gdb_port, hart_id); } else

#ifdef HAS_SCV
#include <scv.h>
#else
#include <scv-tr.h>
using namespace scv_tr;
#endif

#ifndef CWR_SYSTEMC
#define GET_PROP_VALUE(P) P.get_value()
#else
#define GET_PROP_VALUE(P) P.getValue()
#endif

#ifdef _MSC_VER
// not #if defined(_WIN32) || defined(_WIN64) because we have strncasecmp in mingw
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#endif

namespace sysc {
namespace tgfs {
using namespace std;
using namespace iss;
using namespace logging;
using namespace sc_core;

namespace {
iss::debugger::encoder_decoder encdec;

std::array<const char, 4> lvl = {{'U', 'S', 'H', 'M'}};
}

template<typename PLAT>
class core_wrapper_t : public PLAT {
public:
    using reg_t       = typename arch::traits<typename PLAT::core>::reg_t;
    using phys_addr_t = typename arch::traits<typename PLAT::core>::phys_addr_t;
    using heart_state_t = typename PLAT::hart_state_type;
    core_wrapper_t(core_complex *owner)
    : owner(owner) { }

    uint32_t get_mode() { return this->reg.PRIV; }

    inline void set_interrupt_execution(bool v) { this->interrupt_sim = v?1:0; }

    inline bool get_interrupt_execution() { return this->interrupt_sim; }

    heart_state_t &get_state() { return this->state; }

    void notify_phase(iss::arch_if::exec_phase p) override {
        if (p == iss::arch_if::ISTART) owner->sync(this->reg.icount);
    }

    sync_type needed_sync() const override { return PRE_SYNC; }

    void disass_output(uint64_t pc, const std::string instr) override {
        if (!owner->disass_output(pc, instr)) {
            std::stringstream s;
            s << "[p:" << lvl[this->reg.PRIV] << ";s:0x" << std::hex << std::setfill('0')
              << std::setw(sizeof(reg_t) * 2) << (reg_t)this->state.mstatus << std::dec << ";c:"
              << this->reg.icount + this->cycle_offset << "]";
            SCCDEBUG(owner->name())<<"disass: "
                << "0x" << std::setw(16) << std::right << std::setfill('0') << std::hex << pc << "\t\t" << std::setw(40)
                << std::setfill(' ') << std::left << instr << s.str();
        }
    };

    status read_mem(phys_addr_t addr, unsigned length, uint8_t *const data) override {
        if (addr.access && access_type::DEBUG)
            return owner->read_mem_dbg(addr.val, length, data) ? Ok : Err;
        else {
            return owner->read_mem(addr.val, length, data, addr.access && access_type::FETCH) ? Ok : Err;
        }
    }

    status write_mem(phys_addr_t addr, unsigned length, const uint8_t *const data) override {
        if (addr.access && access_type::DEBUG)
            return owner->write_mem_dbg(addr.val, length, data) ? Ok : Err;
        else {
            auto res = owner->write_mem(addr.val, length, data) ? Ok : Err;
            // clear MTIP on mtimecmp write
            if (addr.val == 0x2004000) {
                reg_t val;
                this->read_csr(arch::mip, val);
                if (val & (1ULL << 7)) this->write_csr(arch::mip, val & ~(1ULL << 7));
            }
            return res;
        }
    }

    status read_csr(unsigned addr, reg_t &val) override {
#ifndef CWR_SYSTEMC
        if((addr==arch::time || addr==arch::timeh) && owner->mtime_o.get_interface(0)){
            uint64_t time_val;
            bool ret = owner->mtime_o->nb_peek(time_val);
            if (addr == iss::arch::time) {
                val = static_cast<reg_t>(time_val);
            } else if (addr == iss::arch::timeh) {
                if (sizeof(reg_t) != 4) return iss::Err;
                val = static_cast<reg_t>(time_val >> 32);
            }
            return ret?Ok:Err;
#else
		if((addr==arch::time || addr==arch::timeh)){
			uint64_t time_val = owner->mtime_i.read();
			if (addr == iss::arch::time) {
				val = static_cast<reg_t>(time_val);
			} else if (addr == iss::arch::timeh) {
				if (sizeof(reg_t) != 4) return iss::Err;
				val = static_cast<reg_t>(time_val >> 32);
			}
			return Ok;
#endif
        } else {
            return PLAT::read_csr(addr, val);
        }
    }

    void wait_until(uint64_t flags) override {
        SCCDEBUG(owner->name()) << "Sleeping until interrupt";
        do {
            sc_core::wait(wfi_evt);
        } while (this->reg.pending_trap == 0);
        PLAT::wait_until(flags);
    }

    void local_irq(short id, bool value) {
        reg_t mask = 0;
        switch (id) {
        case 16: // SW
            mask = 1 << 3;
            break;
        case 17: // timer
            mask = 1 << 7;
            break;
        case 18: // external
            mask = 1 << 11;
            break;
        default:
            /* do nothing*/
            break;
        }
        if (value) {
            this->csr[arch::mip] |= mask;
            wfi_evt.notify();
        } else
            this->csr[arch::mip] &= ~mask;
        this->check_interrupt();
        if(value)
            SCCTRACE(owner->name()) << "Triggering interrupt " << id << " Pending trap: " << this->reg.pending_trap;
    }

private:
    core_complex *const owner;
    sc_event wfi_evt;
};

int cmd_sysc(int argc, char *argv[], debugger::out_func of, debugger::data_func df,
             debugger::target_adapter_if *tgt_adapter) {
    if (argc > 1) {
        if (strcasecmp(argv[1], "print_time") == 0) {
            std::string t = sc_time_stamp().to_string();
            of(t.c_str());
            std::array<char, 64> buf;
            encdec.enc_string(t.c_str(), buf.data(), 63);
            df(buf.data());
            return Ok;
        } else if (strcasecmp(argv[1], "break") == 0) {
            sc_time t;
            if (argc == 4) {
                t = scc::parse_from_string(argv[2], argv[3]);
            } else if (argc == 3) {
                t = scc::parse_from_string(argv[2]);
            } else
                return Err;
            // no check needed as it is only called if debug server is active
            tgt_adapter->add_break_condition([t]() -> unsigned {
                SCCTRACE() << "Checking condition at " << sc_time_stamp();
                return sc_time_stamp() >= t ? std::numeric_limits<unsigned>::max() : 0;
            });
            return Ok;
        }
        return Err;
    }
    return Err;
}

using cpu_ptr = std::unique_ptr<iss::arch_if>;
using vm_ptr= std::unique_ptr<iss::vm_if>;

class core_wrapper {
public:
    core_wrapper(core_complex *owner) : owner(owner) { }

    void reset(uint64_t addr){vm->reset(addr);}
    inline void start(){vm->start();}
    inline std::pair<uint64_t, bool> load_file(std::string const& name){ return cpu->load_file(name);};

    std::function<unsigned(void)> get_mode;
    std::function<uint64_t(void)> get_state;
    std::function<bool(void)> get_interrupt_execution;
    std::function<void(bool)> set_interrupt_execution;
    std::function<void(short, bool)> local_irq;

    template<typename PLAT>
    std::tuple<cpu_ptr, vm_ptr> create_core(std::string const& backend, unsigned gdb_port, uint32_t hart_id){
        auto* lcpu = new core_wrapper_t<PLAT>(owner);
        lcpu->set_mhartid(hart_id);
        get_mode = [lcpu]() { return lcpu->get_mode(); };
        get_state = [lcpu]() { return lcpu->get_state().mstatus.backing.val; };
        get_interrupt_execution = [lcpu]() { return lcpu->get_interrupt_execution(); };
        set_interrupt_execution = [lcpu](bool b) { return lcpu->set_interrupt_execution(b); };
        local_irq = [lcpu](short s, bool b) { return lcpu->local_irq(s, b); };
        if(backend == "interp")
            return {cpu_ptr{lcpu}, vm_ptr{iss::interp::create(static_cast<typename PLAT::core*>(lcpu), gdb_port)}};
#ifdef WITH_LLVM
        if(backend == "llvm")
            return {cpu_ptr{lcpu}, vm_ptr{iss::llvm::create(lcpu, gdb_port)}};
#endif
#ifdef WITH_TCC
        if(backend == "tcc")
    s        return {cpu_ptr{lcpu}, vm_ptr{iss::tcc::create(lcpu, gdb_port)}};
#endif
        return {nullptr, nullptr};
    }

    void create_cpu(std::string const& type, std::string const& backend, unsigned gdb_port, uint32_t hart_id){
        CREATE_CORE(tgc_c)
#ifdef CORE_TGC_B
        CREATE_CORE(tgc_b)
#endif
#ifdef CORE_TGC_D
        CREATE_CORE(tgc_d)
#endif
#ifdef CORE_TGC_D_XRB_MAC
        CREATE_CORE(tgc_d_xrb_mac)
#endif
#ifdef CORE_TGC_D_XRB_NN
        CREATE_CORE(tgc_d_xrb_nn)
#endif
        {
            LOG(ERR) << "Illegal argument value for core type: " << type << std::endl;
        }
        auto *srv = debugger::server<debugger::gdb_session>::get();
        if (srv) tgt_adapter = srv->get_target();
        if (tgt_adapter)
            tgt_adapter->add_custom_command(
                {"sysc", [this](int argc, char *argv[], debugger::out_func of,
                                debugger::data_func df) -> int { return cmd_sysc(argc, argv, of, df, tgt_adapter); },
                 "SystemC sub-commands: break <time>, print_time"});

    }

    core_complex * const owner;
    vm_ptr vm{nullptr};
    cpu_ptr cpu{nullptr};
    iss::debugger::target_adapter_if *tgt_adapter{nullptr};
};

struct core_trace {
    //! transaction recording database
    scv_tr_db *m_db{nullptr};
    //! blocking transaction recording stream handle
    scv_tr_stream *stream_handle{nullptr};
    //! transaction generator handle for blocking transactions
    scv_tr_generator<_scv_tr_generator_default_data, _scv_tr_generator_default_data> *instr_tr_handle{nullptr};
    scv_tr_handle tr_handle;
};

SC_HAS_PROCESS(core_complex);// NOLINT
#ifndef CWR_SYSTEMC
core_complex::core_complex(sc_module_name const& name)
: sc_module(name)
, read_lut(tlm_dmi_ext())
, write_lut(tlm_dmi_ext())
{
	init();
}
#endif

void core_complex::init(){
	trc=new core_trace();
    initiator.register_invalidate_direct_mem_ptr([=](uint64_t start, uint64_t end) -> void {
        auto lut_entry = read_lut.getEntry(start);
        if (lut_entry.get_granted_access() != tlm::tlm_dmi::DMI_ACCESS_NONE && end <= lut_entry.get_end_address() + 1) {
            read_lut.removeEntry(lut_entry);
        }
        lut_entry = write_lut.getEntry(start);
        if (lut_entry.get_granted_access() != tlm::tlm_dmi::DMI_ACCESS_NONE && end <= lut_entry.get_end_address() + 1) {
            write_lut.removeEntry(lut_entry);
        }
    });

    SC_THREAD(run);
    SC_METHOD(rst_cb);
    sensitive << rst_i;
    SC_METHOD(sw_irq_cb);
    sensitive << sw_irq_i;
    SC_METHOD(timer_irq_cb);
    sensitive << timer_irq_i;
    SC_METHOD(global_irq_cb);
    sensitive << global_irq_i;
    trc->m_db=scv_tr_db::get_default_db();

	SC_METHOD(forward);
#ifndef CWR_SYSTEMC
	sensitive<<clk_i;
#else
	sensitive<<curr_clk;
	t2t.reset(new scc::tick2time{"t2t"});
	t2t->clk_i(clk_i);
	t2t->clk_o(curr_clk);
#endif
}

core_complex::~core_complex(){
    delete cpu;
    delete trc;
    for (auto *p : plugin_list)
        delete p;
}

void core_complex::trace(sc_trace_file *trf) const {}

void core_complex::before_end_of_elaboration() {
    SCCDEBUG(SCMOD)<<"instantiating iss::arch::tgf with "<<GET_PROP_VALUE(backend)<<" backend";
    // cpu = scc::make_unique<core_wrapper>(this);
    cpu = new core_wrapper(this);
    cpu->create_cpu(GET_PROP_VALUE(core_type), GET_PROP_VALUE(backend), GET_PROP_VALUE(gdb_server_port), GET_PROP_VALUE(mhartid));
    sc_assert(cpu->vm!=nullptr);
    cpu->vm->setDisassEnabled(GET_PROP_VALUE(enable_disass) || trc->m_db != nullptr);
    if (GET_PROP_VALUE(plugins).length()) {
        auto p = util::split(GET_PROP_VALUE(plugins), ';');
        for (std::string const& opt_val : p) {
            std::string plugin_name=opt_val;
            std::string filename{"cycles.txt"};
            std::size_t found = opt_val.find('=');
            if (found != std::string::npos) {
                plugin_name = opt_val.substr(0, found);
                filename = opt_val.substr(found + 1, opt_val.size());
            }
            if (plugin_name == "ic") {
                auto *plugin = new iss::plugin::instruction_count(filename);
                cpu->vm->register_plugin(*plugin);
                plugin_list.push_back(plugin);
            } else if (plugin_name == "ce") {
                auto *plugin = new iss::plugin::cycle_estimate(filename);
                cpu->vm->register_plugin(*plugin);
                plugin_list.push_back(plugin);
            } else if (plugin_name == "pctrace") {
                auto *plugin = new iss::plugin::cov(filename);
                cpu->vm->register_plugin(*plugin);
                plugin_list.push_back(plugin);
            } else {
                std::array<char const*, 1> a{{filename.c_str()}};
                iss::plugin::loader l(plugin_name, {{"initPlugin"}});
                auto* plugin = l.call_function<iss::vm_plugin*>("initPlugin", a.size(), a.data());
                if(plugin){
                    cpu->vm->register_plugin(*plugin);
                    plugin_list.push_back(plugin);
                } else
                    SCCERR(SCMOD) << "Unknown plugin '" << plugin_name << "' or plugin not found";
            }
        }
    }

}

void core_complex::start_of_simulation() {
    quantum_keeper.reset();
    if (GET_PROP_VALUE(elf_file).size() > 0) {
        istringstream is(GET_PROP_VALUE(elf_file));
        string s;
        while (getline(is, s, ',')) {
            std::pair<uint64_t, bool> start_addr = cpu->load_file(s);
#ifndef CWR_SYSTEMC
            if (reset_address.is_default_value() && start_addr.second == true)
                reset_address.set_value(start_addr.first);
#else
            if (start_addr.second == true)
                reset_address=start_addr.first;
#endif
        }
    }
    if (trc->m_db != nullptr && trc->stream_handle == nullptr) {
        string basename(this->name());
        trc->stream_handle = new scv_tr_stream((basename + ".instr").c_str(), "TRANSACTOR", trc->m_db);
        trc->instr_tr_handle = new scv_tr_generator<>("execute", *trc->stream_handle);
    }
}

bool core_complex::disass_output(uint64_t pc, const std::string instr_str) {
    if (trc->m_db == nullptr) return false;
    if (trc->tr_handle.is_active()) trc->tr_handle.end_transaction();
    trc->tr_handle = trc->instr_tr_handle->begin_transaction();
    trc->tr_handle.record_attribute("PC", pc);
    trc->tr_handle.record_attribute("INSTR", instr_str);
    trc->tr_handle.record_attribute("MODE", lvl[cpu->get_mode()]);
    trc->tr_handle.record_attribute("MSTATUS", cpu->get_state());
    trc->tr_handle.record_attribute("LTIME_START", quantum_keeper.get_current_time().value() / 1000);
    return true;
}

void core_complex::forward() {
#ifndef CWR_SYSTEMC
	set_clock_period(clk_i.read());
#else
	set_clock_period(curr_clk.read());

#endif
}

void core_complex::set_clock_period(sc_core::sc_time period) {
	curr_clk = period;
    if (period == SC_ZERO_TIME) cpu->set_interrupt_execution(true);
}

void core_complex::rst_cb() {
    if (rst_i.read()) cpu->set_interrupt_execution(true);
}

void core_complex::sw_irq_cb() { cpu->local_irq(16, sw_irq_i.read()); }

void core_complex::timer_irq_cb() { cpu->local_irq(17, timer_irq_i.read()); }

void core_complex::global_irq_cb() { cpu->local_irq(18, global_irq_i.read()); }

void core_complex::run() {
    wait(SC_ZERO_TIME); // separate from elaboration phase
    do {
        if (rst_i.read()) {
            cpu->reset(GET_PROP_VALUE(reset_address));
            wait(rst_i.negedge_event());
        }
        while (curr_clk.read() == SC_ZERO_TIME) {
            wait(curr_clk.value_changed_event());
        }
        cpu->set_interrupt_execution(false);
        cpu->start();
    } while (cpu->get_interrupt_execution());
    sc_stop();
}

bool core_complex::read_mem(uint64_t addr, unsigned length, uint8_t *const data, bool is_fetch) {
    auto lut_entry = read_lut.getEntry(addr);
    if (lut_entry.get_granted_access() != tlm::tlm_dmi::DMI_ACCESS_NONE &&
        addr + length <= lut_entry.get_end_address() + 1) {
        auto offset = addr - lut_entry.get_start_address();
        std::copy(lut_entry.get_dmi_ptr() + offset, lut_entry.get_dmi_ptr() + offset + length, data);
        quantum_keeper.inc(lut_entry.get_read_latency());
        return true;
    } else {
        tlm::tlm_generic_payload gp;
        gp.set_command(tlm::TLM_READ_COMMAND);
        gp.set_address(addr);
        gp.set_data_ptr(data);
        gp.set_data_length(length);
        gp.set_streaming_width(length);
        sc_time delay=quantum_keeper.get_local_time();
        if (trc->m_db != nullptr && trc->tr_handle.is_valid()) {
            if (is_fetch && trc->tr_handle.is_active()) {
                trc->tr_handle.end_transaction();
            }
            auto preExt = new tlm::scc::scv::tlm_recording_extension(trc->tr_handle, this);
            gp.set_extension(preExt);
        }
        initiator->b_transport(gp, delay);
        SCCTRACE(this->name()) << "read_mem(0x" << std::hex << addr << ") : " << data;
        if (gp.get_response_status() != tlm::TLM_OK_RESPONSE) {
            return false;
        }
        if (gp.is_dmi_allowed()) {
            gp.set_command(tlm::TLM_READ_COMMAND);
            gp.set_address(addr);
            tlm_dmi_ext dmi_data;
            if (initiator->get_direct_mem_ptr(gp, dmi_data)) {
                if (dmi_data.is_read_allowed())
                    read_lut.addEntry(dmi_data, dmi_data.get_start_address(),
                                      dmi_data.get_end_address() - dmi_data.get_start_address() + 1);
                if (dmi_data.is_write_allowed())
                    write_lut.addEntry(dmi_data, dmi_data.get_start_address(),
                                       dmi_data.get_end_address() - dmi_data.get_start_address() + 1);
            }
        }
        return true;
    }
}

bool core_complex::write_mem(uint64_t addr, unsigned length, const uint8_t *const data) {
    auto lut_entry = write_lut.getEntry(addr);
    if (lut_entry.get_granted_access() != tlm::tlm_dmi::DMI_ACCESS_NONE &&
        addr + length <= lut_entry.get_end_address() + 1) {
        auto offset = addr - lut_entry.get_start_address();
        std::copy(data, data + length, lut_entry.get_dmi_ptr() + offset);
        quantum_keeper.inc(lut_entry.get_read_latency());
        return true;
    } else {
        write_buf.resize(length);
        std::copy(data, data + length, write_buf.begin()); // need to copy as TLM does not guarantee data integrity
        tlm::tlm_generic_payload gp;
        gp.set_command(tlm::TLM_WRITE_COMMAND);
        gp.set_address(addr);
        gp.set_data_ptr(write_buf.data());
        gp.set_data_length(length);
        gp.set_streaming_width(length);
        sc_time delay=quantum_keeper.get_local_time();
        if (trc->m_db != nullptr && trc->tr_handle.is_valid()) {
            auto preExt = new tlm::scc::scv::tlm_recording_extension(trc->tr_handle, this);
            gp.set_extension(preExt);
        }
        initiator->b_transport(gp, delay);
        quantum_keeper.set(delay);
        SCCTRACE() << "write_mem(0x" << std::hex << addr << ") : " << data;
        if (gp.get_response_status() != tlm::TLM_OK_RESPONSE) {
            return false;
        }
        if (gp.is_dmi_allowed()) {
            gp.set_command(tlm::TLM_READ_COMMAND);
            gp.set_address(addr);
            tlm_dmi_ext dmi_data;
            if (initiator->get_direct_mem_ptr(gp, dmi_data)) {
                if (dmi_data.is_read_allowed())
                    read_lut.addEntry(dmi_data, dmi_data.get_start_address(),
                                      dmi_data.get_end_address() - dmi_data.get_start_address() + 1);
                if (dmi_data.is_write_allowed())
                    write_lut.addEntry(dmi_data, dmi_data.get_start_address(),
                                       dmi_data.get_end_address() - dmi_data.get_start_address() + 1);
            }
        }
        return true;
    }
}

bool core_complex::read_mem_dbg(uint64_t addr, unsigned length, uint8_t *const data) {
    auto lut_entry = read_lut.getEntry(addr);
    if (lut_entry.get_granted_access() != tlm::tlm_dmi::DMI_ACCESS_NONE &&
        addr + length <= lut_entry.get_end_address() + 1) {
        auto offset = addr - lut_entry.get_start_address();
        std::copy(lut_entry.get_dmi_ptr() + offset, lut_entry.get_dmi_ptr() + offset + length, data);
        quantum_keeper.inc(lut_entry.get_read_latency());
        return true;
    } else {
        tlm::tlm_generic_payload gp;
        gp.set_command(tlm::TLM_READ_COMMAND);
        gp.set_address(addr);
        gp.set_data_ptr(data);
        gp.set_data_length(length);
        gp.set_streaming_width(length);
        return initiator->transport_dbg(gp) == length;
    }
}

bool core_complex::write_mem_dbg(uint64_t addr, unsigned length, const uint8_t *const data) {
    auto lut_entry = write_lut.getEntry(addr);
    if (lut_entry.get_granted_access() != tlm::tlm_dmi::DMI_ACCESS_NONE &&
        addr + length <= lut_entry.get_end_address() + 1) {
        auto offset = addr - lut_entry.get_start_address();
        std::copy(data, data + length, lut_entry.get_dmi_ptr() + offset);
        quantum_keeper.inc(lut_entry.get_read_latency());
        return true;
    } else {
        write_buf.resize(length);
        std::copy(data, data + length, write_buf.begin()); // need to copy as TLM does not guarantee data integrity
        tlm::tlm_generic_payload gp;
        gp.set_command(tlm::TLM_WRITE_COMMAND);
        gp.set_address(addr);
        gp.set_data_ptr(write_buf.data());
        gp.set_data_length(length);
        gp.set_streaming_width(length);
        return initiator->transport_dbg(gp) == length;
    }
}
} /* namespace SiFive */
} /* namespace sysc */
