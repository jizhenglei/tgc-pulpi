

// #pragma once

// #include "scc/tlm_target.h"
// #include "sifive/gen/uart_regs.h"
// #include "util/bit_field.h"

// namespace vpvper::dummy {

// class Dummy : public sc_core::sc_module, public scc::tlm_target<> {
//  public:
//   // constructor
//   Dummy()
//       : sc_core::sc_module(sc_core::sc_module_name{"Dummy"}),
//         scc::tlm_target<>(clk),
//         NAMEDD(regs, vpvper::sifive::uart_regs) {
//     regs->registerResources(*this);

//     regs->txdata.set_write_cb([this](scc::sc_register<uint32_t> &reg,
//                                      uint32_t data,
//                                      sc_core::sc_time d) -> bool {
//       std::cout << "udma: hello there\n";
//       return true;
//     });
//   }
//   // destructor
//   virtual ~Dummy() override {}

//  protected:
//   sc_core::sc_time clk{sc_core::SC_ZERO_TIME};
//   std::unique_ptr<vpvper::sifive::uart_regs> regs{};
// };
// }  // namespace vpvper::dummy


/*
 * Copyright (c) 2019 -2022 MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #ifndef _PULPISSIMO_UDMA_H_
// #define _PULPISSIMO_UDMA_H_
// #include "scc/tlm_target.h"

// namespace pulpissimo {
// namespace gen {
// class udma_regs;
// // class spi_channel_regs;

// }

// class udma : public sc_core::sc_module, public scc::tlm_target<> {
// public:
//     sc_core::sc_in<sc_core::sc_time> clk_i{"clk_i"};
//     sc_core::sc_in<bool> rst_i{"rst_i"};
    
//     udma(sc_core::sc_module_name nm);
//     virtual ~udma() override;

// protected:
//     void clock_cb();
//     void reset_cb();
//     sc_core::sc_time clk;
//     std::unique_ptr<gen::udma_regs> regs;
//     // std::unique_ptr<gen::spim_regs> regs;
//     uint32_t rx_bu_ptr;
//     uint32_t tx_bu_ptr;
//     uint32_t cmd_bu_ptr;
//     // std::array<bool, 32> spim_cmd{1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//     uint32_t op = spim_cmd[28]*1+spim_cmd[29]*2+spim_cmd[30]*4+spim_cmd[31]*8;
//     void transmit();
//     void udma::spim_cmd(std::array<bool, 32> spim_cmd);
//     bool quad = false;
//     uint16_t chip_select;
//     bool lsb;
//     bool cpol;
//     bool cpha;
//     uint32_t command_size;
//     uint32_t dummy_cycle;


// };

// } /* namespace pulpissimo */





// #endif /* _PULPISSIMO_UDMA_H_ */