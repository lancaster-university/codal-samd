#ifndef __STM32_ASSERT_H
#define __STM32_ASSERT_H

#ifdef __cplusplus
extern "C"
#endif
    void
    target_panic(int statusCode);

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : target_panic(919))
#else
#define assert_param(expr) ((void)0U)
#endif

#endif
