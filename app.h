/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H
#include <stdio.h>

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)

/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

#define TIMER_ID_RESTART        78
#define TIMER_ID_FACTORY_RESET  77
#define TIMER_ID_BUTTON_POLL    49
#define TIMER_ID_GET_DCD        20
#define TIMER_ID_APPKEY_ADD     21
#define TIMER_ID_APPKEY_BIND    22
#define TIMER_ID_PUB_SET        23
#define TIMER_ID_SUB_ADD        24
// max number of SIG models in the DCD
#define MAX_SIG_MODELS    20
// max number of vendor models in the DCD
#define MAX_VENDOR_MODELS 4

#define LIGHT_CTRL_GRP_ADDR     0xC001
#define LIGHT_STATUS_GRP_ADDR   0xC002
#define VENDOR_GRP_ADDR         0xC003
#define VENDOR_ID_INVALID       0xFFFF



#define PROVISION_ATTN_TIME_DISABLED       0  //attention timer in seconds
#define ADDRESS_ASSIGN_AUTO                0  //let the stack assign addresses automatically.

#define PAGE0 0


#define CONFIGURE_VENDOR_MODEL
/* models used by simple light example (on/off only)
 * The beta SDK 1.0.1 and 1.1.0 examples are based on these
 * */
#define LIGHT_MODEL_ID            0x1000 // Generic On/Off Server
#define SWITCH_MODEL_ID           0x1001 // Generic On/Off Client

/*
 * Lightness models used in the dimming light example of 1.2.0 SDK
 * */
#define DIM_LIGHT_MODEL_ID              0x1300 // Light Lightness Server
#define DIM_SWITCH_MODEL_ID             0x1302 // Light Lightness Client

typedef struct
{
  uint16_t model_id;
  uint16_t vendor_id;
} tsModel;

/* struct for storing the content of one element in the DCD */
typedef struct
{
  uint16_t SIG_models[MAX_SIG_MODELS];
  uint8_t numSIGModels;

  tsModel vendor_models[MAX_VENDOR_MODELS];
  uint8_t numVendorModels;
}tsDCD_ElemContent;


typedef struct
{
  // model bindings to be done. for simplicity, all models are bound to same appkey in this example
  // (assuming there is exactly one appkey used and the same appkey is used for all model bindings)
  tsModel bind_model[4];
  uint8_t num_bind;
  uint8_t num_bind_done;

  // publish addresses for up to 4 models
  tsModel pub_model[4];
  uint16_t pub_address[4];
  uint8_t num_pub;
  uint8_t num_pub_done;

  // subscription addresses for up to 4 models
  tsModel sub_model[4];
  uint16_t sub_address[4];
  uint8_t num_sub;
  uint8_t num_sub_done;

}tsConfig;

/* this struct is used to help decoding the raw DCD data */
typedef struct
{
  uint16_t companyID;
  uint16_t productID;
  uint16_t version;
  uint16_t replayCap;
  uint16_t featureBitmask;
  uint8_t payload[1];
} tsDCD_Header;

/* this struct is used to help decoding the raw DCD data */
typedef struct
{
  uint16_t location;
  uint8_t numSIGModels;
  uint8_t numVendorModels;
  uint8_t payload[1];
} tsDCD_Elem;

typedef struct
{
  uint16_t err;
  const char *pShortDescription;
} tsErrCode;

#define D_INPUT_MAX             32
typedef struct
{
  uint8_t error:1;
  uint8_t finis:1;
  uint8_t index:5;
  uint8_t buffer[D_INPUT_MAX];
  uint8_t key[D_INPUT_MAX/2];
} tsInput;

typedef struct
{
  uint8_t prov_start:1;
  uint8_t oob_static:1;
  uint8_t oob_blink:1;
}tsBit;

typedef union
{
  tsBit bit;
  uint8_t all;
}tuInput;

#define STATUS_OK                      0
#define STATUS_BUSY                    0x181
#define SOFT_TIMER_SINGLE_SHOT         0
#define SOFT_TIMER_REPEATING           1
/***********************************************************************************************//**
 * \defgroup app Application Code
 * \brief Sample Application Implementation
 **************************************************************************************************/
#define ELEMENT(x)                     x
#define FRIENDSHIP_CREDENTIALS_NONE    0
#define PUB_TTL                        3
#define PUB_PERIOD_MS                  0
#define PUB_RETRANS_COUNT              0
#define PUB_RETRANS_INTERVAL_MS        50
#define KEY_INDEX_INVALID            0xFF
#define ADDRESS_INVALID            0xFFFF
/***************************************************************************//**
 * Application Init.
 ******************************************************************************/
void app_init(void);

/***************************************************************************//**
 * Application Process Action.
 ******************************************************************************/
void app_process_action(void);

#endif // APP_H
