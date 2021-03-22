#pragma once

#include "nrf_assert.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Crypto library in bootloader case does not use dynamic allocation */
#define NRF_CRYPTO_ALLOC(size)                                                                                                             \
    NULL;                                                                                                                                  \
    ASSERT(0)
#define NRF_CRYPTO_ALLOC_ON_STACK(size)                                                                                                    \
    NULL;                                                                                                                                  \
    ASSERT(0)
#define NRF_CRYPTO_FREE(ptr) (void)ptr;

#ifdef __cplusplus
}
#endif
