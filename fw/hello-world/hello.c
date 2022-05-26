#include <stdint.h>
#include <stdio.h>
#include <unistd.h>





/** \enum rt_spim_wordsize_e
 * \brief Wordsize of the SPI bitstream elements.
 *
 * This is used to know how the endianness must be applied.
 */
typedef enum {
  RT_SPIM_WORDSIZE_8 = 0,     /*!< Each element is 8 bits. Thus the endianness has no effect. */
  RT_SPIM_WORDSIZE_32 = 1     /*!< Each element is 32 bits. The way each element is stored in memory can then be specified with the endianness. */
} rt_spim_wordsize_e;

/** \struct rt_spim_conf_t
 * \brief SPI master configuration structure.
 *
 * This structure is used to pass the desired SPI master configuration to the runtime when opening a device.
 */
typedef struct __rt_spim_conf_s {
  int max_baudrate;       /*!< Maximum baudrate for the SPI bitstream which can be used with the opened device . */
  char wordsize;          /*!< Wordsize of the elements in the bitstream. Can be RT_SPIM_WORDSIZE_8 for 8 bits data or RT_SPIM_WORDSIZE_32 for 32 bits data. This is used to interpret the endianness. */
  char big_endian;        /*!< If 1, the elements are stored in memory in a big-endian way, i.e. the most significant byte is stored at the lowest address. This is taken into account only if the wordsize is 32 bits. */
  char polarity;          /*!< Polarity of the clock. */
  char phase;             /*!< Phase of the clock. */
  signed char cs_gpio;    /*!< If it is different from -1, the specified number is used to drive a GPIO which is used as a chip select for the SPI device. The cs field is then ignored. */
  signed char cs;         /*!< If cs_gpio is -1, the normal chip select pins are used and this field specifies which one to use for the device. */
  signed char id;         /*!< If it is different from -1, this specifies on which SPI interface the device is connected. */
} rt_spim_conf_t;

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

int main() {
  rt_spim_conf_t conf;
  // Get default configuration
  rt_spim_conf_init(&conf);
}
