/*
 * Copyright (c) 2019 -2022 MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udma.h"
#include "gen/udma.h"
#include "gen/spi_channel.h"
#include "scc/utilities.h"
#include "tlm/scc/tlm_extensions.h"
#include <scc/utilities.h>

#include "scc/utilities.h"
#include <tlm/scc/signal_initiator_mixin.h>
#include <tlm/scc/signal_target_mixin.h>
#include <generic/tlm_extensions.h>

// namespace pulpissimo {
// SC_HAS_PROCESS(udma);// NOLINT

// udma::udma(sc_core::sc_module_name nm)
// : sc_core::sc_module(nm)
// , scc::tlm_target<>(clk)
// , NAMEDD(regs, gen::udma_regs) {
//     regs->registerResources(*this);
//     SC_METHOD(clock_cb);
//     sensitive << clk_i;
//     SC_METHOD(reset_cb);
//     sensitive << rst_i;
//     // regs->i_uart.STATUS.set_write_cb([this](scc::sc_register<uint32_t>&, uint32_t const& v, sc_core::sc_time t)-> bool {return true;});
//     // regs->i_uart.VALID.set_write_cb([this](scc::sc_register<uint32_t>&, uint32_t const& v, sc_core::sc_time t)-> bool {return true;});
//     // regs->CTRL_CFG_CG.set_write_cb([this](scc::sc_register<uint32_t>&, uint32_t v, sc_core::sc_time t)-> bool {
//         // std::array<bool, 32> spim_cmd{1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//         // for(int i=0; i<=31; i++){
//         //     spim_cmd[i]=1;
//         // }
//         // auto op = spim_cmd[28]*1+spim_cmd[29]*2+spim_cmd[30]*4+spim_cmd[31]*8;
//         // std::cout << "SPIM: hello there\n";
//         // printf("spi_m command is %05d\n", op);
//         // std::cout << "Value of str is : " << op << std::endl;
//         // return true;});

//     regs->i_spi.SPIM_RX_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t rx_buffer_add, sc_core::sc_time t) -> bool {
//         reg.put(rx_buffer_add);
//         return true;}
//         );

//     regs->i_spi.SPIM_RX_SADDR.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
//         if(regs->i_spi.r_SPIM_RX_CFG.EN == 1){
            
//             for(auto i = regs->i_spi.r_SPIM_RX_SADDR; i < regs -> i_spi.SPIM_RX_SIZE; i++){
//                 if(regs->i_spi.r_SPIM_RX_CFG.CLR == 1 || regs->i_spi.r_SPIM_RX_CFG.EN == 0){
//                 data = i;
                
//             }
            
//         }
//         rx_bu_ptr = data;
//         return true;
//     }});

//     regs->i_spi.SPIM_RX_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t rx_buffer_size, sc_core::sc_time t) -> bool {
//         if(0 < rx_buffer_size < 0xfffff){   //Buffer size in bytes. (1MBytes maximum)
//             reg.put(rx_buffer_size);
//             }
//             else return 0;
//         return true;
//     });

//     regs->i_spi.SPIM_RX_SIZE.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t rx_buffer_size, sc_core::sc_time t) -> bool {
        
//         auto rxbu_size_rem = regs->i_spi.r_SPIM_RX_SIZE - rx_bu_ptr;
//         return true;}
//     );

//     // regs->SPIM_RX_CFG.set_read_cb([this](scc::sc_register<gen::spi_channel_regs::SPIM_RX_CFG_t> const & reg, uint32_t& rx_cfg, sc_core::sc_time t)-> bool {
//     //     gen::spi_channel_regs::SPIM_RX_CFG_t st = reg.get();

//     regs->i_spi.SPIM_TX_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t tx_buffer_add, sc_core::sc_time t) -> bool {
//         reg.put(tx_buffer_add);
//         return true;
//     });

//     regs->i_spi.SPIM_TX_SADDR.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
//         if(regs->i_spi.r_SPIM_TX_CFG.EN == 1){
//             for(auto i = regs->i_spi.r_SPIM_TX_SADDR; i < regs -> i_spi.r_SPIM_RX_SIZE; ++i ){
//                 if(regs->i_spi.r_SPIM_TX_CFG.CLR == 1 || regs->i_spi.r_SPIM_TX_CFG.EN == 0){
//                 data = i;    
//                 }
//             }    
//         }
//         tx_bu_ptr = data; 
//         return true;
       
//     });

//     regs->i_spi.SPIM_TX_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t tx_buffer_size, sc_core::sc_time t) -> bool {
//         if(0 < tx_buffer_size < 0xfffff){   //Buffer size in bytes. (1MBytes maximum)
//             reg.put(tx_buffer_size);
//             return true;
//             }
//         else
//             return false;
//     });

//     regs->i_spi.SPIM_TX_SIZE.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t txbu_size_rem, sc_core::sc_time t) -> bool {        
//         txbu_size_rem = regs->i_spi.r_SPIM_TX_SIZE - tx_bu_ptr;
//         return true;
//     });

//     regs->i_spi.SPIM_CMD_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t cmd_buffer_add, sc_core::sc_time t) -> bool {
//         reg.put(cmd_buffer_add);
//         return true;
//     });

//     regs->i_spi.SPIM_CMD_SADDR.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
//         if(regs->i_spi.r_SPIM_TX_CFG.EN == 1){
//             for(auto i = regs->i_spi.r_SPIM_CMD_SADDR; i < regs -> i_spi.r_SPIM_CMD_SIZE; ++i ){
//                 if(regs->i_spi.r_SPIM_RX_CFG.CLR == 1|| regs->i_spi.r_SPIM_TX_CFG.EN == 0){
//                     data = i;
//                 }
//             }
//         }
//         cmd_bu_ptr = data;
//         return 0;
//     });

//     regs->i_spi.SPIM_CMD_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t cmd_buffer_size, sc_core::sc_time t) -> bool {
//         if(0 < cmd_buffer_size < 0xfffff){   //Buffer size in bytes. (1MBytes maximum)
//             reg.put(cmd_buffer_size);
//             return true;
//             }
//             return false;
//     });

//     regs->i_spi.SPIM_CMD_SIZE.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t cmdbu_size_rem, sc_core::sc_time t) -> bool {
//         cmdbu_size_rem = regs->i_spi.r_SPIM_CMD_SIZE - rx_bu_ptr;
//         return true;
//     });



    

// }

// udma::~udma() {} // NOLINT

// void udma::clock_cb() { this->clk = clk_i.read(); }

// void udma::reset_cb() {
//     if (rst_i.read()) {
//         regs->reset_start();
//     } else {
//         regs->reset_stop();
//     }
// }



// void udma::spi_cmd(std::array<bool, 32> spim_cmd){ 

//     // auto op = spim_cmd[28]*1+spim_cmd[29]*2+spim_cmd[30]*4+spim_cmd[31]*8;
//     if (op=0){
//         if(!spim_cmd[10]&&!spim_cmd[9]){
//             // CPOL0
//             // CPHA0
//         }
//         if(spim_cmd[10]&&!spim_cmd[9]){
//             // CPOL1
//             // CPHA0
//         }
//         if(!spim_cmd[10]&&spim_cmd[9]){
//             // CPOL0
//             // CPHA1
//         }
//         if(spim_cmd[10]&&spim_cmd[9]){
//             // CPOL1
//             // CPHA1
//         }

//     } 
//     if (op=1){
//         if(!spim_cmd[0]&&!spim_cmd[1]){
//             chip_select = 0;
//             // CSN0

//         }
//         if(!spim_cmd[0]&&spim_cmd[1]){
//             chip_select = 1;
//             // CSN1
//         }
//         if(spim_cmd[0]&&!spim_cmd[1]){
//             chip_select = 2;
//             // CSN2
//         }
//         if(spim_cmd[0]&&spim_cmd[1]){
//             chip_select = 3;
//             // CSN3
//         }
//     }
//     if (op=2){
//         if(spim_cmd[27]){
//             quad = true;
//             // quad
//         }
//         if(!spim_cmd[27]){
//             quad = false;
//             // not quad
//         }
//         if(spim_cmd[26]){
//             lsb = true;
//             // LSB first
//         }
//         if(!spim_cmd[26]){
//             lsb = false;
//             // MSB first
//         }
//         for(int i=16; i<=19; i++){
//             if(spim_cmd[i]){
//                 command_size=spim_cmd[16]+spim_cmd[17]*2+spim_cmd[18]*4+spim_cmd[19]*8;
//             }
//         }
       
//         if(command_size<16){
//             for(int j=0; j<16; j++){
//                 if(spim_cmd[j]=NULL){
//                     spim_cmd[j]=0;
//                 }
//             }
//         }
//         // VALUE: The data to be send. MSB must always be at bit 15 also if SIZE is lower than 16.
    
//     }

//     if(op=4){
//         dummy_cycle=spim_cmd[16]+spim_cmd[17]*2+spim_cmd[18]*4+spim_cmd[19]*8+spim_cmd[20]*16;

//     }

//     if(op=5){
        
//         // wait

//     }

//     if(op=6){

//     }





// }

// void udma::transmit(){
//     uint8_t tx_data;
//     // tlm::tlm_phase phase(tlm::BEGIN_REQ);
    
// 	// sc_core::sc_time delay(SC_ZERO_TIME);
//     // tlm::tlm_generic_payload trans;
    

//     auto *trans = tlm::scc::tlm_signal_gp<>::create();

//     auto *ext = new sysc::tlm_signal_spi_extension();

//     ext->tx.m2s_data = tx_data;
//     // ext->tx.m2s_data_valid = data_valid;
//     ext->tx.s2m_data_valid = false;

//     trans->set_command(tlm::TLM_WRITE_COMMAND);
//     // trans->set_address(regs->i_spi.SPIM_TX_SADDR);




// }






// }
 /* namespace pulpissimo */