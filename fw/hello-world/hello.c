#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
// #include "spim-v3.c"
// #include "include/rt/data/rt_data_spim.h"
// #include "include/rt/rt_spim.h"
#include "include_b/rt/rt_data.h"
// #include "include/rt/rt_debug.h"
// #include "include/rt/rt_freq.h"
// #include "include_a/hal/udma/udma_v3.h"
// #include "include/rt.h"
// #include "include/rt/rt_periph.h"
// #include "include/rt/rt_irq.h"
// #include "include_a/archi/chips/pulpissimo/properties.h"
// #include "kernel/alloc.c"
// #include "kernel/dev.c"
// #include "include/rt/data/rt_data_spim.h"
// #include "include/rt/rt_perf.h"
// #include "include/rt/rt_periph.h"
// #include "include_a/archi/udma/udma_v3.h"





// #include "encoding.h"
// #include "platform.h"

// int factorial(int i) {
//   volatile int result = 1;
//   for (int ii = 1; ii <= i; ii++) {
//     result = result * ii;
//   }
//   return result;
// }

// int main() {
//   *(uint32_t*)(GPIO_CTRL_ADDR + GPIO_IOF_SEL) &= ~IOF0_UART0_MASK;
//   *(uint32_t*)(GPIO_CTRL_ADDR + GPIO_IOF_EN) |= IOF0_UART0_MASK;
//   volatile int result = factorial (10);
//   printf("Factorial is %d\n", result);
//   printf("End of execution");

//   return 0;
// }


// /** \enum rt_spim_wordsize_e
//  * \brief Wordsize of the SPI bitstream elements.
//  *
//  * This is used to know how the endianness must be applied.
//  */
// typedef enum {
//   RT_SPIM_WORDSIZE_8 = 0,     /*!< Each element is 8 bits. Thus the endianness has no effect. */
//   RT_SPIM_WORDSIZE_32 = 1     /*!< Each element is 32 bits. The way each element is stored in memory can then be specified with the endianness. */
// } rt_spim_wordsize_e;

// /** \struct rt_spim_conf_t
//  * \brief SPI master configuration structure.
//  *
//  * This structure is used to pass the desired SPI master configuration to the runtime when opening a device.
//  */
// typedef struct __rt_spim_conf_s {
//   int max_baudrate;       /*!< Maximum baudrate for the SPI bitstream which can be used with the opened device . */
//   char wordsize;          /*!< Wordsize of the elements in the bitstream. Can be RT_SPIM_WORDSIZE_8 for 8 bits data or RT_SPIM_WORDSIZE_32 for 32 bits data. This is used to interpret the endianness. */
//   char big_endian;        /*!< If 1, the elements are stored in memory in a big-endian way, i.e. the most significant byte is stored at the lowest address. This is taken into account only if the wordsize is 32 bits. */
//   char polarity;          /*!< Polarity of the clock. */
//   char phase;             /*!< Phase of the clock. */
//   signed char cs_gpio;    /*!< If it is different from -1, the specified number is used to drive a GPIO which is used as a chip select for the SPI device. The cs field is then ignored. */
//   signed char cs;         /*!< If cs_gpio is -1, the normal chip select pins are used and this field specifies which one to use for the device. */
//   signed char id;         /*!< If it is different from -1, this specifies on which SPI interface the device is connected. */
// } rt_spim_conf_t;


// typedef struct __rt_spim_s rt_spim_t;

// typedef struct __rt_spim_s {
//   int max_baudrate;
// // #if defined(UDMA_VERSION) && UDMA_VERSION >= 1
//   unsigned int cfg;
//   char cs;
// // #endif
//   char wordsize;
//   char big_endian;
//   signed char cs_gpio;
//   char channel;

// // #if !defined(UDMA_VERSION)

// //   int div;
// //   rt_spim_t *next;

// // #elif defined(UDMA_VERSION) && UDMA_VERSION >= 2

//   char byte_align;
//   unsigned char div;
//   char polarity;
//   char phase;

// // #elif defined(UDMA_VERSION) && UDMA_VERSION == 1

// //   unsigned char div;
// //   char polarity;
// //   char phase;

// // #endif

// } rt_spim_t;

int main() {
  rt_spim_conf_t conf;
  // Get default configuration
  rt_spim_conf_init(&conf);

  conf.max_baudrate = 1000000;
  // SPI interface identifier as the Pulp chip can have
  // several interfaces
  conf.id = 0; 
  // Chip select
  conf.cs = 1;

  printf("New, End of execution");

  // Then open the device
  rt_spim_t *spim = rt_spim_open(NULL, &conf, NULL);
  if (spim == NULL) return -1;


}

 



static int __rt_spim_open_count[ARCHI_UDMA_NB_SPIM];
void rt_spim_conf_init(rt_spim_conf_t *conf)
{
  conf->wordsize = RT_SPIM_WORDSIZE_8;
  conf->big_endian = 0;
  conf->max_baudrate = 10000000;
  conf->cs_gpio = -1;
  conf->cs = -1;
  conf->id = -1;
  conf->polarity = 0;
  conf->phase = 0;
}

rt_spim_t *rt_spim_open(char *dev_name, rt_spim_conf_t *conf, rt_event_t *event)
{
  int irq = rt_irq_disable();

  rt_spim_conf_t def_conf;

  if (conf == NULL)
  {
    conf = &def_conf;
    rt_spim_conf_init(conf);
  }

  int channel = -1;

  if (conf->id != -1)
  {  
    rt_trace(RT_TRACE_DEV_CTRL, "[SPIM] Opening spim device (id: %d)\n", conf->id);
    channel = ARCHI_UDMA_SPIM_ID(conf->id);
  }
  else if (dev_name != NULL)
  {
    rt_trace(RT_TRACE_DEV_CTRL, "[SPIM] Opening spim device (name: %s)\n", dev_name);
  
    rt_dev_t *dev = rt_dev_get(dev_name);
    if (dev == NULL) goto error;

    channel = dev->channel;
  }

  if (channel == -1) goto error;

  rt_spim_t *spim = rt_alloc(RT_ALLOC_FC_DATA, sizeof(rt_spim_t));
  if (spim == NULL) goto error;

  spim->channel = channel;

  spim->wordsize = conf->wordsize;
  spim->big_endian = conf->big_endian;
  spim->polarity = conf->polarity;
  spim->phase = conf->phase;
  spim->max_baudrate = conf->max_baudrate;
  spim->cs = conf->cs;
  spim->byte_align = __rt_spim_get_byte_align(conf->wordsize, conf->big_endian);

  int div = __rt_spi_get_div(spim->max_baudrate);
  spim->div = div;

  spim->cfg = SPI_CMD_CFG(div, conf->polarity, conf->phase);

  int id = channel - ARCHI_UDMA_SPIM_ID(0);

  __rt_spim_open_count[id]++;

  if (__rt_spim_open_count[id] == 1)
  {
    plp_udma_cg_set(plp_udma_cg_get() | (1<<channel));
    __rt_udma_callback[channel] = __rt_spim_handle_eot;
    __rt_udma_callback_data[channel] = &__rt_spim_periph[id];
    soc_eu_fcEventMask_setEvent(ARCHI_SOC_EVENT_PERIPH_EVT_BASE(channel) + ARCHI_UDMA_SPIM_EOT_EVT);
  }

  rt_irq_restore(irq);

  return spim;

error:
  rt_warning("[SPIM] Failed to open spim device\n");
  return NULL;
}

