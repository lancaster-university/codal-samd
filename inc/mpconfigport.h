#include "sam.h"

#ifdef SAMD21
#define PROTOTYPE_SERCOM_SPI_M_SYNC_CLOCK_FREQUENCY 48000000
#endif
#ifdef SAMD51
#define PROTOTYPE_SERCOM_SPI_M_SYNC_CLOCK_FREQUENCY 100000000
#endif

#ifndef __cplusplus
typedef unsigned char           bool;
#endif
