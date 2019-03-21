#ifndef PLATFORM_INCLUDES
#define PLATFORM_INCLUDES

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "sam.h"

#define PROCESSOR_WORD_TYPE uint32_t

#ifdef __cplusplus
extern "C" {
#endif

void target_panic(int statusCode);
void wait_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#define MBED_ASSERT CODAL_ASSERT
#define MBED_ERROR(msg) CODAL_ASSERT(0)
#define MBED_WEAK __attribute__((weak))

#endif
