#ifndef _PULPISSIMO_SPIM_H_
#define _PULPISSIMO_SPIM_H_
#include "scc/tlm_target.h"


//  typedef struct spim_conf_s {
//   int pending;       /*!< Maximum baudrate for the SPI bitstream which can be used with the opened device . */
//   int datasize;          /*!< Wordsize of the elements in the bitstream. Can be RT_SPIM_WORDSIZE_8 for 8 bits data or RT_SPIM_WORDSIZE_32 for 32 bits data. This is used to interpret the endianness. */
//   int continous;        /*!< If 1, the elements are stored in memory in a big-endian way, i.e. the most significant byte is stored at the lowest address. This is taken into account only if the wordsize is 32 bits. */
// //   char polarity;          /*!< Polarity of the clock. */
// //   char phase;             /*!< Phase of the clock. */
// //   signed char cs_gpio;    /*!< If it is different from -1, the specified number is used to drive a GPIO which is used as a chip select for the SPI device. The cs field is then ignored. */
// //   signed char cs;         /*!< If cs_gpio is -1, the normal chip select pins are used and this field specifies which one to use for the device. */
// //   signed char id;         /*!< If it is different from -1, this specifies on which SPI interface the device is connected. */
// } spim_conf_t;

namespace pulpissimo {
namespace gen {
// class spim_regs;
class spi_channel_regs;
}
class spi_m : public sc_core::sc_module, public scc::tlm_target<> {
public:
    sc_core::sc_in<sc_core::sc_time> clk_i{"clk_i"};
    sc_core::sc_in<bool> rst_i{"rst_i"};
    spi_m(sc_core::sc_module_name nm);
    virtual ~spi_m() override;

protected:
    // tlm::scc::tlm_signal_bool_opt_out _mosi_o;
    // tlm::scc::tlm_signal_bool_opt_in _miso_i;

    void clock_cb();
    void reset_cb();
    sc_core::sc_time clk;
    std::unique_ptr<gen::spi_channel_regs> regs;

    uint32_t rx_bu_ptr;
    uint32_t tx_bu_ptr;
    uint32_t cmd_bu_ptr;
    std::array<bool, 32> spim_cmd{1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint32_t op = spim_cmd[28]*1+spim_cmd[29]*2+spim_cmd[30]*4+spim_cmd[31]*8;
    void transmit_data();
    // void receive_data(tlm::scc::tlm_signal_gp<> &gp, sc_core::sc_time &delay);
    void spi_cmd(std::array<bool, 32> spim_cmd);
    bool quad = false;
    uint16_t chip_select;
    bool lsb;
    bool cpol;
    bool cpha;
    uint32_t command_size;
    uint32_t dummy_cycle;
    void spim_open();
    void spim_send();





    
};

} /* namespace pulpissimo */

#endif /* _PULPISSIMO_SOC_CTRL_H_ */