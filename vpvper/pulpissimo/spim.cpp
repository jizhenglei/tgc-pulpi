#include "spim.h"
#include "gen/spi_channel.h"
#include "scc/utilities.h"
#include "tlm/scc/tlm_extensions.h"

#include <scc/utilities.h>
#include <scc/tlm_target.h>


#include <tlm/scc/signal_initiator_mixin.h>
#include <tlm/scc/signal_target_mixin.h>
#include <generic/tlm_extensions.h>

namespace pulpissimo {

SC_HAS_PROCESS(spi_m);// NOLINT

spi_m::spi_m(sc_core::sc_module_name nm)
: sc_core::sc_module(nm)
, scc::tlm_target<>(clk)
// , NAMED(_mosi_o)
// , NAMED(_miso_i)
, NAMEDD(regs, gen::spi_channel_regs) {
    regs->registerResources(*this);
    SC_METHOD(clock_cb);
    sensitive << clk_i;
    SC_METHOD(reset_cb);
    sensitive << rst_i;

    // SC_THREAD(transmit_data);
    // _miso_i.register_nb_transport(
    //     [this](tlm::scc::tlm_signal_gp<bool> &gp, tlm::tlm_phase &phase, sc_core::sc_time &delay) -> tlm::tlm_sync_enum {
    //         this->receive_data(gp, delay);
    //         return tlm::TLM_COMPLETED;
    //     });

    regs->SPIM_RX_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t rx_buffer_add, sc_core::sc_time t) -> bool {
        reg.put(rx_buffer_add);
        return true;}
        );

    regs->SPIM_RX_SADDR.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
        if(regs->r_SPIM_RX_CFG.EN == 1){
            
            for(auto i = regs->r_SPIM_RX_SADDR; i < regs -> SPIM_RX_SIZE; i++){
                if(regs->r_SPIM_RX_CFG.CLR == 1 || regs->r_SPIM_RX_CFG.EN == 0){
                data = i;
                
                }
            rx_bu_ptr = data;  
            }
            
        }
        return true;
    });

    regs->SPIM_RX_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t rx_buffer_size, sc_core::sc_time t) -> bool {
        if(0 < rx_buffer_size < 0xfffff){   //Buffer size in bytes. (1MBytes maximum)
            reg.put(rx_buffer_size);
            }
            else return 0;
        return true;
    });

    regs->SPIM_RX_SIZE.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t rx_buffer_size, sc_core::sc_time t) -> bool {
        
        auto rxbu_size_rem = regs->r_SPIM_RX_SIZE - rx_bu_ptr;
        return true;}
    );

    // regs->SPIM_RX_CFG.set_read_cb([this](scc::sc_register<gen::spi_channel_regs::SPIM_RX_CFG_t> const & reg, uint32_t& rx_cfg, sc_core::sc_time t)-> bool {
    //     gen::spi_channel_regs::SPIM_RX_CFG_t st = reg.get();

    regs->SPIM_TX_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t tx_buffer_add, sc_core::sc_time t) -> bool {
        reg.put(tx_buffer_add);
        return true;
    });

    regs->SPIM_TX_SADDR.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
        if(regs->r_SPIM_TX_CFG.EN == 1){
            for(auto i = regs->r_SPIM_TX_SADDR; i < regs -> r_SPIM_TX_SIZE; ++i ){
                if(regs->r_SPIM_TX_CFG.CLR == 1 || regs->r_SPIM_TX_CFG.EN == 0){
                data = i;    
                }
            }    
        }
        tx_bu_ptr = data; 
        return true;
       
    });

    regs->SPIM_TX_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t tx_buffer_size, sc_core::sc_time t) -> bool {
        if(0 < tx_buffer_size < 0xfffff){   //Buffer size in bytes. (1MBytes maximum)
            reg.put(tx_buffer_size);
            return true;
            }
        else
            return false;
    });

    regs->SPIM_TX_SIZE.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t txbu_size_rem, sc_core::sc_time t) -> bool {        
        txbu_size_rem = regs->r_SPIM_TX_SIZE - tx_bu_ptr;
        return true;
    });

    regs->SPIM_CMD_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t cmd_buffer_add, sc_core::sc_time t) -> bool {
        reg.put(cmd_buffer_add);
        return true;
    });

    regs->SPIM_CMD_SADDR.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
        if(regs->r_SPIM_TX_CFG.EN == 1){

            for(auto i = regs->r_SPIM_CMD_SADDR; i < regs -> r_SPIM_CMD_SIZE; ++i ){
                if(regs->r_SPIM_RX_CFG.CLR == 1|| regs->r_SPIM_TX_CFG.EN == 0){
                    data = i;
                }
            }
        }
        cmd_bu_ptr = data;
        return 0;
    });

    regs->SPIM_CMD_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t cmd_buffer_size, sc_core::sc_time t) -> bool {
        if(0 < cmd_buffer_size < 0xfffff){   //Buffer size in bytes. (1MBytes maximum)
            reg.put(cmd_buffer_size);
            return true;
            }
        else
            return false;
    });

    regs->SPIM_CMD_SIZE.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t cmdbu_size_rem, sc_core::sc_time t) -> bool {
        cmdbu_size_rem = regs->r_SPIM_CMD_SIZE - rx_bu_ptr;
        return true;
    });

    regs->SPIM_CMD_CFG.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
        if(regs->r_SPIM_CMD_CFG.EN == 1){std::cout << "Opening spim device" << std::endl;}
        reg.put(data);
        return true;
    });

    regs->SPIM_RX_CFG.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
        reg.put(data);
        return true;
    });

    regs->SPIM_TX_CFG.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time t) -> bool {
        reg.put(data);
        return true;
    });





    

}

spi_m::~spi_m() {} // NOLINT

void spi_m::clock_cb() { this->clk = clk_i.read(); }

void spi_m::reset_cb() {
    if (rst_i.read()) {
        regs->reset_start();
    } else {
        regs->reset_stop();
    }
}

void spi_m::spi_cmd(std::array<bool, 32> spim_cmd){ 

    // auto op = spim_cmd[28]*1+spim_cmd[29]*2+spim_cmd[30]*4+spim_cmd[31]*8;
    if (op=0){
        if(!spim_cmd[10]&&!spim_cmd[9]){
            // CPOL0
            // CPHA0
        }
        if(spim_cmd[10]&&!spim_cmd[9]){
            // CPOL1
            // CPHA0
        }
        if(!spim_cmd[10]&&spim_cmd[9]){
            // CPOL0
            // CPHA1
        }
        if(spim_cmd[10]&&spim_cmd[9]){
            // CPOL1
            // CPHA1
        }

    } 
    if (op=1){
        if(!spim_cmd[0]&&!spim_cmd[1]){
            chip_select = 0;
            // CSN0

        }
        if(!spim_cmd[0]&&spim_cmd[1]){
            chip_select = 1;
            // CSN1
        }
        if(spim_cmd[0]&&!spim_cmd[1]){
            chip_select = 2;
            // CSN2
        }
        if(spim_cmd[0]&&spim_cmd[1]){
            chip_select = 3;
            // CSN3
        }
    }
    if (op=2){
        if(spim_cmd[27]){
            quad = true;
            // quad
        }
        if(!spim_cmd[27]){
            quad = false;
            // not quad
        }
        if(spim_cmd[26]){
            lsb = true;
            // LSB first
        }
        if(!spim_cmd[26]){
            lsb = false;
            // MSB first
        }
        for(int i=16; i<=19; i++){
            if(spim_cmd[i]){
                command_size=spim_cmd[16]+spim_cmd[17]*2+spim_cmd[18]*4+spim_cmd[19]*8-1;
            }
        }
       
        if(command_size<16){
            for(int j=0; j<16; j++){
                if(spim_cmd[j]=NULL){
                    spim_cmd[j]=0;
                }
            }
        }
        // VALUE: The data to be send. MSB must always be at bit 15 also if SIZE is lower than 16.
    
    }

    if(op=4){
        dummy_cycle=spim_cmd[16]+spim_cmd[17]*2+spim_cmd[18]*4+spim_cmd[19]*8+spim_cmd[20]*16;

    }

    if(op=5){
        
        // wait

    }

    if(op=6){

    }





}



// void spi_m::transmit_data(){
//     uint8_t tx_data;
//     bool data_valid;
//     // tlm::tlm_phase phase(tlm::BEGIN_REQ);
    
// 	sc_core::sc_time delay(SC_ZERO_TIME);
//     // tlm::tlm_generic_payload trans;
    

//     // auto *trans = tlm::scc::tlm_signal_gp<>::create();

//     auto *ext = new sysc::tlm_signal_spi_extension();

//     tlm::tlm_generic_payload trans;
	

// 	trans.set_command(tlm::TLM_WRITE_COMMAND);
// 	trans.set_address(regs->r_SPIM_TX_SADDR);
// 	trans.set_data_ptr();
// 	trans.set_data_length();
// 	trans.set_streaming_width();
	
// 	trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

	

// 	// init_socket->b_transport(trans, delay);

//     ext->tx.m2s_data = tx_data;
//     ext->tx.m2s_data_valid = data_valid;
//     ext->tx.s2m_data_valid = false;

//     // trans->set_command(tlm::TLM_WRITE_COMMAND);
//     // trans->set_address(regs->i_spi.SPIM_TX_SADDR);





// }

// void spi_m::spim_open()
// {
//     regs->r_SPIM_CMD_CFG.CLR = 1;
//     regs->r_SPIM_CMD_CFG.EN = 1;
//     regs->r_SPIM_CMD_CFG.PENDING = 1;
//     regs->r_SPIM_CMD_CFG.DATASIZE = 01;
//     regs->r_SPIM_CMD_CFG.CONTINOUS = 1;
    
    
//         // "[SPIM] Opening spim device (id: %d)\n"
//     // std::cout << "Opening spim device" << std::endl;
//     if(regs->r_SPIM_CMD_CFG.EN == 1){std::cout << "Opening spim device" << std::endl;}
    
    



// }

// void spi_m::spim_send(){


    
// }


}
 /* namespace pulpissimo */
