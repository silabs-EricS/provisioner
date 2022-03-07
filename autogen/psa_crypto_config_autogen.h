// This is an autogenerated config file, any changes to this file will be overwritten

#ifndef PSA_CRYPTO_CONFIG_AUTOGEN_H
#define PSA_CRYPTO_CONFIG_AUTOGEN_H

#define PSA_WANT_KEY_TYPE_AES
#define PSA_WANT_ALG_CCM
#define PSA_WANT_ALG_ECB_NO_PADDING
#define PSA_WANT_ALG_CMAC
#define PSA_WANT_KEY_TYPE_ECC_PUBLIC_KEY
#define PSA_WANT_KEY_TYPE_ECC_KEY_PAIR
#define PSA_WANT_ECC_SECP_R1_256
#define PSA_WANT_ALG_ECDH
#define PSA_WANT_ALG_HMAC
#define PSA_WANT_KEY_TYPE_HMAC
#define PSA_WANT_ALG_SHA_1
#define PSA_WANT_ALG_SHA_256
#define MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG
#define MBEDTLS_PSA_ACCEL_ALG_SHA_1
#define MBEDTLS_PSA_ACCEL_ALG_SHA_224
#define MBEDTLS_PSA_ACCEL_ALG_SHA_256
#define MBEDTLS_PSA_ACCEL_KEY_TYPE_AES
#define MBEDTLS_PSA_ACCEL_ALG_ECB_NO_PADDING
#define MBEDTLS_PSA_ACCEL_ALG_CBC_NO_PADDING
#define MBEDTLS_PSA_ACCEL_ALG_CBC_PKCS7
#define MBEDTLS_PSA_ACCEL_ALG_CTR
#define MBEDTLS_PSA_ACCEL_ALG_CFB
#define MBEDTLS_PSA_ACCEL_ALG_OFB
#define MBEDTLS_PSA_ACCEL_ALG_GCM
#define MBEDTLS_PSA_ACCEL_ALG_CCM
#define MBEDTLS_PSA_ACCEL_ALG_HMAC
#define MBEDTLS_PSA_ACCEL_ALG_CMAC
#define MBEDTLS_PSA_ACCEL_ALG_CBC_MAC
#define MBEDTLS_PSA_ACCEL_KEY_TYPE_ECC_KEY_PAIR
#define MBEDTLS_PSA_ACCEL_KEY_TYPE_ECC_PUBLIC_KEY
#define MBEDTLS_PSA_ACCEL_ALG_ECDSA
#define MBEDTLS_PSA_ACCEL_ALG_ECDH
#define MBEDTLS_PSA_ACCEL_ECC_SECP_R1_192
#define MBEDTLS_PSA_ACCEL_ECC_SECP_R1_256

#include "sl_btmesh_config.h"
#define MBEDTLS_PSA_KEY_SLOT_COUNT (2 + (4 + 4 * (SL_BTMESH_CONFIG_MAX_NETKEYS + SL_BTMESH_CONFIG_MAX_FRIENDSHIPS) + 2 * SL_BTMESH_CONFIG_MAX_PROV_SESSIONS + SL_BTMESH_CONFIG_ITS_KEY_CACHE_SIZE) + 1 + SL_PSA_KEY_USER_SLOT_COUNT)
#define SL_PSA_ITS_MAX_FILES (0x240 + 1 + SL_PSA_ITS_USER_MAX_FILES)

#include "config-device-acceleration.h"
#if defined(SL_SE_ASSUME_FW_AT_LEAST_1_2_10) || defined(SL_SE_ASSUME_FW_AT_LEAST_2_1_7)
    #define MBEDTLS_PSA_ACCEL_ECC_MONTGOMERY_255
    #undef MBEDTLS_PSA_BUILTIN_ECC_MONTGOMERY_255
    #if !(defined(MBEDTLS_ECP_DP_SECP192R1_ENABLED) \
          || defined(MBEDTLS_ECP_DP_SECP224R1_ENABLED) \
          || defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED) \
          || defined(MBEDTLS_ECP_DP_SECP384R1_ENABLED) \
          || defined(MBEDTLS_ECP_DP_SECP521R1_ENABLED))
        #undef MBEDTLS_PSA_BUILTIN_KEY_TYPE_ECC_KEY_PAIR
        #undef MBEDTLS_PSA_BUILTIN_KEY_TYPE_ECC_PUBLIC_KEY
        #undef MBEDTLS_PSA_BUILTIN_ALG_ECDH
    #endif /* !MBEDTLS_ECP_DP_SECPxR1_ENABLED */
#endif /* SL_SE_ASSUME_FW_AT_LEAST_x */

#endif
