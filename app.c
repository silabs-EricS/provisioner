/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
#include "em_common.h"
#include "app_assert.h"
#include "app_log.h"
#include "sl_status.h"
#include "app.h"

#include "sl_btmesh_api.h"
#include "sl_bt_api.h"

//#define D_OOB

/// Advertising Provisioning Bearer
#define PB_ADV                         0x1
/// GATT Provisioning Bearer
#define PB_GATT                        0x2

/*******************************************************************************
 * Global variables
 ******************************************************************************/
/// number of active Bluetooth connections
static uint8_t num_connections = 0;

static bool init_done = false;

enum {
  init,
  scanning,
  connecting,
  provisioning,
  provisioned,
  waiting_dcd,
  waiting_appkey_ack,
  waiting_bind_ack,
  waiting_pub_ack,
  waiting_sub_ack
} state;

#ifdef USE_FIXED_KEYS
const uint8_t fixed_netkey[16] = {0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3};
const uint8_t fixed_appkey[16] = {4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7};
#endif

static const uint8_t oob_data[16] =
{ 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32_t)32768)
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

uint8_t netkey_id = 0;//KEY_INDEX_INVALID;
uint8_t appkey_id = 0;//KEY_INDEX_INVALID;
uint8_t ask_user_input = 0;
uint16_t provisionee_address = ADDRESS_INVALID;//0xFFFF;

tsInput input;
tuInput input_request;

uint8_t config_retrycount = 0;
uuid_128 _uuid_copy;
uint32_t  handle;


uint8_t ascii2hex(uint8_t a)
{
  uint8_t ren = 0;
  if((a >= '0') && (a <= '9')){
    ren = a - '0';
  }else if((a >= 'A') && (a <= 'F')){
    ren = a - 'A' + 0x0A;
  }else if((a >= 'a') && (a <= 'f')){
    ren = a - 'a' + 0x0A;
  }
  return ren;
}

void ascii2hex_array(uint8_t *a)
{
  for (int i = 0; i < 32; i+=2){
    input.key[i/2] = 16 * ascii2hex(a[i + 0]) + ascii2hex(a[i + 1]);
  }
}

/*******************************************************************************
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  app_log("BT mesh Provisioner initialized\r\n");
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

int RETARGET_ReadChar(void)
{
  char ch;
  sl_status_t status = sl_iostream_getchar(SL_IOSTREAM_STDIN, &ch);
  if (status != SL_STATUS_OK) {
    return EOF;
  }

  return ch;
}

static void  button_poll()
{
  sl_status_t sc;
  int key;

  if(input_request.all == 0){
    return;
  }

  /* provisioning can be accepted/rejected either by pushbuttons PB1/PB0
   * or alternatively by entering 'y' / 'n' on keyboard */
  key = RETARGET_ReadChar();

  //no key input, return
  if (key == -1)
    return;

  if(input_request.bit.prov_start){
    if (key == 'y') {
        input_request.bit.prov_start = 0;
      app_log("\r\nSending prov request\r\n");
      sl_bt_system_set_soft_timer(0, TIMER_ID_BUTTON_POLL, 0);
      sc = sl_btmesh_prov_create_provisioning_session(netkey_id, _uuid_copy, 30);
      if (sc != SL_STATUS_OK) {
        app_log("Failed call to create provision session. (0x%04X)\r\n", sc);
        return;
      }
#ifdef D_OOB
      sc = sl_btmesh_prov_set_oob_requirements (_uuid_copy,
                                                0x00 /* public key */,
                                                sl_btmesh_node_auth_method_flag_static/*|sl_btmesh_node_auth_method_flag_output|sl_btmesh_node_auth_method_flag_input*/ /* auth_method*/,
                                                sl_btmesh_node_oob_output_action_flag_blink /*output_actions*/,
                                                sl_btmesh_node_oob_input_action_flag_push /*input_actions*/,
                                                1 /*output_size*/,
                                                1 /*input_size*/);
      if (sc == SL_STATUS_OK) {
        app_log("Provision set oob requirements!\r\n");
      } else {
        app_log("Failed to set provision oob requirements. Error: 0x%04X\r\n", sc);
      }
#endif
      sc = sl_btmesh_prov_set_provisioning_suspend_event(1);
      if (sc != SL_STATUS_OK) {
        app_log("Failed call to set provisioning suspend event. (0x%04X)\r\n", sc);
        return;
      }

      #ifndef PROVISION_OVER_GATT
      // provisioning using ADV bearer (this is the default)
      sc = sl_btmesh_prov_provision_adv_device(_uuid_copy);
      if (sc == SL_STATUS_OK) {
        state = provisioning;
        app_log("provisioning...\r\n");
      } else {
        app_log("Failed call to provision node. (0x%04X)\r\n", sc);
      }
      #else
      // provisioning using GATT bearer. First we must open a connection to the remote device
      if(gecko_cmd_le_gap_open(bt_address, bt_address_type)->result == 0)
      {
        app_log("trying to open a connection\r\n");
      }
      else
      {
        app_log("le_gap_open failed\r\n");
      }
      state = connecting;

      #endif
    } else if (key == 'n') {
      input_request.bit.prov_start = 0;
      sl_bt_system_set_soft_timer(0, TIMER_ID_BUTTON_POLL, 0);
    }
  } else if (input_request.bit.oob_static){
      input.error = 1;

      if (((key >= '0') && (key <= '9'))
          || ((key >= 'a') && (key <= 'f'))
          || ((key >= 'A') && (key <= 'F'))){
        input.error = 0;
        input.buffer[input.index] = key;
        input.index++;
        if(input.index == 0/*5bits only, overlap, D_INPUT_MAX*/){
          input_request.bit.oob_static = 0;
          sl_bt_system_set_soft_timer(0, TIMER_ID_BUTTON_POLL, 0);
          ascii2hex_array(input.buffer);

          app_log("\r\nInput finish!\r\n");
          for(int i = 0; i < D_INPUT_MAX/2; i++)
            app_log("%02X", input.key[i]);
          app_log("\r\n");

          sc = sl_btmesh_prov_send_oob_auth_response(_uuid_copy, 16, input.key);
          if (sc == SL_STATUS_OK) {
            app_log("Provision send_oob_static ...\r\n");
          } else {
            app_log("Failed call to send_oob_static. Error: 0x%04X\r\n", sc);
          }

        }
      }
      if(input.error == 1){
        input.error = 0;
        input.index = 0;
        app_log("\r\nInput error, try again!\r\n");
      }
  }else if (input_request.bit.oob_blink){
      if ((key >= '0') && (key <= '9')){
        app_log("\r\n");
        input_request.bit.oob_blink = 0;
        sl_bt_system_set_soft_timer(0, TIMER_ID_BUTTON_POLL, 0);
        memset(input.key, 0, 16);
        input.key[15] = key - '0';
        sc = sl_btmesh_prov_send_oob_auth_response(_uuid_copy, 16, input.key);
        if (sc == SL_STATUS_OK) {
          app_log("Provision send_oob_blink ...\r\n");
        } else {
          app_log("Failed call to send_oob_blink. Error: 0x%04X\r\n", sc);
        }
      }else{
        app_log("\r\nInput error, try again!\r\n");
      }
  }
}

// DCD content of the last provisioned device. (the example code decodes up to two elements, but
// only the primary element is used in the configuration to simplify the code)
tsDCD_ElemContent _sDCD_Prim;
tsDCD_ElemContent _sDCD_2nd; /* second DCD element is decoded if present, but not used for anything (just informative) */
uint8_t _dcd_raw[256]; // raw content of the DCD received from remote node
uint8_t _dcd_raw_len = 0;
// config data to be sent to last provisioned node:
tsConfig _sConfig;

/* function for decoding one element inside the DCD. Parameters:
 *  pElem: pointer to the beginning of element in the raw DCD data
 *  pDest: pointer to a struct where the decoded values are written
 * */
static void DCD_decode_element(tsDCD_Elem *pElem, tsDCD_ElemContent *pDest)
{
  uint16_t *pu16;
  int i;

  memset(pDest, 0, sizeof(*pDest));

  pDest->numSIGModels = pElem->numSIGModels;
  pDest->numVendorModels = pElem->numVendorModels;

  app_log("Num sig models: %d\r\n", pDest->numSIGModels );
  app_log("Num vendor models: %d\r\n", pDest->numVendorModels);

  if(pDest->numSIGModels > MAX_SIG_MODELS)
  {
    app_log("ERROR: number of SIG models in DCD exceeds MAX_SIG_MODELS (%u) limit!\r\n", MAX_SIG_MODELS);
    return;
  }
  if(pDest->numVendorModels > MAX_VENDOR_MODELS)
  {
    app_log("ERROR: number of VENDOR models in DCD exceeds MAX_VENDOR_MODELS (%u) limit!\r\n", MAX_VENDOR_MODELS);
    return;
  }

  // set pointer to the first model:
  pu16 = (uint16_t *)pElem->payload;

  // grab the SIG models from the DCD data
  for(i=0;i<pDest->numSIGModels;i++)
  {
    pDest->SIG_models[i] = *pu16;
    pu16++;
    app_log("model ID: %4.4x\r\n", pDest->SIG_models[i]);
  }

  // grab the vendor models from the DCD data
  for (i = 0; i < pDest->numVendorModels; i++) {
    pDest->vendor_models[i].vendor_id = *pu16;
    pu16++;
    pDest->vendor_models[i].model_id = *pu16;
    pu16++;

    app_log("vendor ID: %4.4x, model ID: %4.4x\r\n", pDest->vendor_models[i].vendor_id, pDest->vendor_models[i].model_id);
  }
}

static void DCD_decode()
{
  tsDCD_Header *pHeader;
  tsDCD_Elem *pElem;
  uint8_t byte_offset;

  pHeader = (tsDCD_Header *)&_dcd_raw;

  app_log("DCD: company ID %4.4x, Product ID %4.4x\r\n", pHeader->companyID, pHeader->productID);

  pElem = (tsDCD_Elem *)pHeader->payload;

  // decode primary element:
  DCD_decode_element(pElem, &_sDCD_Prim);

  // check if DCD has more than one element by calculating where we are currently at the raw
  // DCD array and compare against the total size of the raw DCD:
  byte_offset = 10 + 4 + pElem->numSIGModels * 2 + pElem->numVendorModels * 4; // +10 for DCD header, +4 for header in the DCD element

  if(byte_offset < _dcd_raw_len)
  {
    // set elem pointer to the beginning of 2nd element:
    pElem = (tsDCD_Elem *)&(_dcd_raw[byte_offset]);

    app_log("Decoding 2nd element (just informative, not used for anything)\r\n");
    DCD_decode_element(pElem, &_sDCD_2nd);
  }
}

/*
 * Add one publication setting to the list of configurations to be done
 * */
static void config_pub_add(uint16_t model_id, uint16_t vendor_id, uint16_t address)
{
  _sConfig.pub_model[_sConfig.num_pub].model_id = model_id;
  _sConfig.pub_model[_sConfig.num_pub].vendor_id = vendor_id;
  _sConfig.pub_address[_sConfig.num_pub] = address;
  _sConfig.num_pub++;
}

/*
 * Add one subscription setting to the list of configurations to be done
 * */
static void config_sub_add(uint16_t model_id, uint16_t vendor_id, uint16_t address)
{
  _sConfig.sub_model[_sConfig.num_sub].model_id = model_id;
  _sConfig.sub_model[_sConfig.num_sub].vendor_id = vendor_id;
  _sConfig.sub_address[_sConfig.num_sub] = address;
  _sConfig.num_sub++;
}

/*
 * Add one appkey/model bind setting to the list of configurations to be done
 * */
static void config_bind_add(uint16_t model_id, uint16_t vendor_id, uint16_t netkey_id, uint16_t appkey_id)
{
  _sConfig.bind_model[_sConfig.num_bind].model_id = model_id;
  _sConfig.bind_model[_sConfig.num_bind].vendor_id = vendor_id;
  _sConfig.num_bind++;
}

/*
 * This function scans for the SIG models in the DCD that was read from a freshly provisioned node.
 * Based on the models that are listed, the publish/subscribe addresses are added into a configuration list
 * that is later used to configure the node.
 *
 * This example configures generic on/off client and lightness client to publish
 * to "light control" group address and subscribe to "light status" group address.
 *
 * Similarly, generic on/off server and lightness server (= the light node) models
 * are configured to subscribe to "light control" and publish to "light status" group address.
 *
 * Alternative strategy for automatically filling the configuration data would be to e.g. use the product ID from the DCD.
 *
 * NOTE: this example only checks the primary element of the node. Other elements are ignored.
 * */
static void config_check()
{
  int i;

  memset(&_sConfig, 0, sizeof(_sConfig));

  // scan the SIG models in the DCD data
  for(i=0;i<_sDCD_Prim.numSIGModels;i++)
  {
    if(_sDCD_Prim.SIG_models[i] == SWITCH_MODEL_ID)
    {
      config_pub_add(SWITCH_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
      config_sub_add(SWITCH_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
      config_bind_add(SWITCH_MODEL_ID, 0xFFFF, 0, 0);
    }
    else if(_sDCD_Prim.SIG_models[i] == LIGHT_MODEL_ID)
    {
      config_pub_add(LIGHT_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
      config_sub_add(LIGHT_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
      config_bind_add(LIGHT_MODEL_ID, 0xFFFF, 0, 0);
    }
    else if(_sDCD_Prim.SIG_models[i] == DIM_SWITCH_MODEL_ID)
    {
      config_pub_add(DIM_SWITCH_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
      config_sub_add(DIM_SWITCH_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
      config_bind_add(DIM_SWITCH_MODEL_ID, 0xFFFF, 0, 0);
    }
    else if(_sDCD_Prim.SIG_models[i] == DIM_LIGHT_MODEL_ID)
    {
      config_pub_add(DIM_LIGHT_MODEL_ID, 0xFFFF, LIGHT_STATUS_GRP_ADDR);
      config_sub_add(DIM_LIGHT_MODEL_ID, 0xFFFF, LIGHT_CTRL_GRP_ADDR);
      config_bind_add(DIM_LIGHT_MODEL_ID, 0xFFFF, 0, 0);
    }

  }

#ifdef CONFIGURE_VENDOR_MODEL
  // scan the vendor models found in the DCD
  for(i=0;i<_sDCD_Prim.numVendorModels;i++)
  {
    // this example only handles vendor model with vendor ID 0x02FF (Silabs) and model ID 0xABCD.
    // if such model found, configure it to publish/subscribe to a single group address
    config_pub_add(_sDCD_Prim.vendor_models[i].model_id, _sDCD_Prim.vendor_models[i].vendor_id, VENDOR_GRP_ADDR);
    config_sub_add(_sDCD_Prim.vendor_models[i].model_id, _sDCD_Prim.vendor_models[i].vendor_id, VENDOR_GRP_ADDR);
    // using single appkey to bind all models. It could be also possible to use different appkey for the
    // vendor models
    config_bind_add(_sDCD_Prim.vendor_models[i].model_id, 0x1111, 0, 0);
  }

#endif
}

static void config_retry(uint8_t timer_handle){
  /* maximum number of retry attempts is limited to 5 in this example. If the limit
   * is reached, then there is probably something wrong with the configuration and
   * there is no point to do try again anymore */
  const uint8_t max_retrycount = 5;

  if(config_retrycount > max_retrycount){
      app_log("ERROR: max limit of configuration retries reached\r\n");
  }
  else{
    config_retrycount++;
  }

  app_log(" trying again, attempt %u/%u\r\n", config_retrycount, max_retrycount);
  sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(500), timer_handle, SOFT_TIMER_REPEATING);

}

static void trigger_next_state(uint8_t timer_handle){
  // when moving to new state in the configuration state machine, the retry counter is cleared:
  config_retrycount = 0;

  sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(100), timer_handle, 1);
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(struct sl_bt_msg *evt)
{
  sl_status_t sc;
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      sc = sl_btmesh_node_reset();
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to reset node\r\n",
                 (int)sc);
      // Perform a full reset by erasing PS storage.
      //sc = sl_bt_nvm_erase_all();
      //app_assert(sc == SL_STATUS_OK,
      //           "[E: 0x%04x] Failed to erase NVM\r\n",
      //          (int)sc);

      // Print boot message.
      app_log("Bluetooth stack booted: v%d.%d.%d-b%d\r\n",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);

      // Initialize Mesh stack in Node operation mode,
      // wait for initialized event
      app_log("Provisioner init\r\n");
      sc = sl_btmesh_prov_init();
      app_assert_status_f(sc, "Failed to init node\n");
      break;
    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_system_soft_timer_id:
      switch(evt->data.evt_system_soft_timer.handle){
        case TIMER_ID_BUTTON_POLL:
          button_poll();
          break;

        case TIMER_ID_GET_DCD:
        {
          app_log("TIMER_ID_GET_DCD, netkey_id: %d, address: 0x%04X \r\n",netkey_id,provisionee_address);

          // clear the old DCD from memory before requesting new one:
          _dcd_raw_len = 0;

          sc = sl_btmesh_config_client_get_dcd(netkey_id,
                                               provisionee_address,
                                               PAGE0,
                                               &handle
                                               );
          if (sc){
            app_log("sl_btmesh_config_client_get_dcd failed with error: 0x%04X, addr: 0x%02X\r\n", sc, provisionee_address);
            sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(1000), TIMER_ID_GET_DCD, SOFT_TIMER_REPEATING);
          } else {
            app_log("requesting DCD from the node...\r\n");
            state = waiting_dcd;
          }
        }
        break;

        case TIMER_ID_APPKEY_ADD:
         {
           sc = sl_btmesh_config_client_add_appkey(netkey_id,
                                                    provisionee_address,
                                                    appkey_id,
                                                    netkey_id,
                                                    &handle
                                                    );
           if (sc) {
             app_log("Appkey deployment failed. addr 0x%02X, error: 0x%04X\r\n", provisionee_address, sc);
             // try again:
             config_retry(TIMER_ID_APPKEY_ADD);
           } else {
             app_log("Deploying appkey to node 0x%4.4X\r\n", provisionee_address);
             state = waiting_appkey_ack;
           }
         }
           break;

        case TIMER_ID_APPKEY_BIND:
        {
          uint16_t vendor_id;
          uint16_t model_id;

          // take the next model from the list of models to be bound with application key.
          // for simplicity, the same appkey is used for all models but it is possible to also use several appkeys
          model_id = _sConfig.bind_model[_sConfig.num_bind_done].model_id;
          vendor_id = _sConfig.bind_model[_sConfig.num_bind_done].vendor_id;

          app_log("APP BIND, config %d/%d:: model %4.4x key index %x\r\n", _sConfig.num_bind_done+1, _sConfig.num_bind, model_id, appkey_id);

          sc = sl_btmesh_config_client_bind_model( netkey_id,
                                                   provisionee_address,
                                                   0, // element index
                                                   vendor_id,
                                                   model_id,
                                                   appkey_id,
                                                   &handle
                                                   );

          if(sc){
            app_log("sl_btmesh_config_client_bind_model failed with result 0x%04X\r\n", sc);
            // try again:
            config_retry(TIMER_ID_APPKEY_BIND);
          }else {
            app_log(" waiting bind ack\r\n");
            state = waiting_bind_ack;
          }
        }
        break;

        case TIMER_ID_PUB_SET:
        {
          uint16_t vendor_id;
          uint16_t model_id;
          uint16_t pub_address;

          // get the next model/address pair from the configuration list:
          model_id = _sConfig.pub_model[_sConfig.num_pub_done].model_id;
          vendor_id = _sConfig.pub_model[_sConfig.num_pub_done].vendor_id;
          pub_address = _sConfig.pub_address[_sConfig.num_pub_done];

          app_log("PUB SET, config %d/%d: model %4.4x -> address %4.4x\r\n", _sConfig.num_pub_done+1, _sConfig.num_pub, model_id, pub_address);

          sc = sl_btmesh_config_client_set_model_pub( netkey_id,
                                                      provisionee_address,
                                                      ELEMENT(0),//0, /* element index */
                                                      vendor_id,
                                                      model_id,
                                                      pub_address,
                                                      appkey_id,
                                                      FRIENDSHIP_CREDENTIALS_NONE,//0, /* friendship credential flag */
                                                      PUB_TTL,//3, /* Publication time-to-live value */
                                                      PUB_PERIOD_MS,//0, /* period = NONE */
                                                      PUB_RETRANS_COUNT,//0, /* Publication retransmission count */
                                                      PUB_RETRANS_INTERVAL_MS,//50  /* Publication retransmission interval */
                                                      &handle
                                                      );

          if (sc){
            app_log("sl_btmesh_config_client_set_model_pub failed with result 0x%04X\r\n", sc);
          } else {
            app_log(" waiting pub ack\r\n");
            state = waiting_pub_ack;
          }
        }
        break;

        case TIMER_ID_SUB_ADD:
        {
          uint16_t vendor_id = VENDOR_ID_INVALID;//0xFFFF;
          uint16_t model_id;
          uint16_t sub_address;

          // get the next model/address pair from the configuration list:
          model_id = _sConfig.sub_model[_sConfig.num_sub_done].model_id;
          vendor_id = _sConfig.sub_model[_sConfig.num_sub_done].vendor_id;
          sub_address = _sConfig.sub_address[_sConfig.num_sub_done];

          app_log("SUB ADD, config %d/%d: model %4.4x -> address %4.4x\r\n", _sConfig.num_sub_done+1, _sConfig.num_sub, model_id, sub_address);

          sc = sl_btmesh_config_client_add_model_sub(netkey_id,
                                                     provisionee_address,
                                                     ELEMENT(0),//0, /* element index */
                                                     vendor_id,
                                                     model_id,
                                                     sub_address,
                                                     &handle
													 );

          if(sc){
            app_log("sl_btmesh_config_client_add_model_sub failed with result 0x%04X\r\n", sc);
          } else {
            app_log(" waiting sub ack\r\n");
            state = waiting_sub_ack;
          }
        }
        break;
      }
      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/**************************************************************************//**
 * Bluetooth Mesh stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth Mesh stack.
 *****************************************************************************/
void sl_btmesh_on_event(sl_btmesh_msg_t *evt)
{
  sl_status_t sc;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_btmesh_evt_prov_initialized_id:
      {
      //app_log("sl_btmesh_evt_prov_initialized_id\r\n");
      sl_btmesh_evt_prov_initialized_t *initialized_evt;
      initialized_evt = (sl_btmesh_evt_prov_initialized_t *)&(evt->data);
      app_log("networks: %x\r\n", initialized_evt->networks);
      app_log("address: %x\r\n", initialized_evt->address);
      app_log("iv_index: %x\r\n", (unsigned int)initialized_evt->iv_index);


      if (initialized_evt->networks > 0) {
        app_log("network keys already exist\r\n");
        netkey_id = 0;
        appkey_id = 0;
      } else {
        app_log("Creating a new netkey\r\n");


        #ifdef USE_FIXED_KEYS
        sc = sl_btmesh_prov_create_network(netkey_id,
                                           16,
                                           fixed_netkey);
        #else
        sc = sl_btmesh_prov_create_network(netkey_id,
                                           0,
                                           (const uint8_t *)"");
        #endif

        if (sc == SL_STATUS_OK ) {
          app_log("Success, netkey id = %x\r\n", netkey_id);
        } else {
          app_log("Failed to create new netkey. Error: 0x%04x\r\n", sc);
        }

        app_log("Creating a new appkey\r\n");


        uint8_t max_application_key_size = 16;
        size_t application_key_len;
        uint8_t application_key[max_application_key_size];
        #ifdef USE_FIXED_KEYS
        sc = sl_btmesh_prov_create_appkey(netkey_id,
                                          appkey_id,
                                          16,
                                          fixed_appkey,
                                          max_application_key_size,
                                          &application_key_len,
                                          application_key);
        #else
        sc = sl_btmesh_prov_create_appkey(netkey_id,
                                          appkey_id,
                                          0,
                                          (const uint8_t *)"",
                                          max_application_key_size,
                                          &application_key_len,
                                          application_key);
        #endif

        if (sc == SL_STATUS_OK ) {
          app_log("Success, appkey_id = %x\r\n", appkey_id);
          app_log("Appkey: ");
          for (uint8_t i = 0; i < application_key_len; ++i) {
            app_log("0x%02X ", application_key[i]);
          }
          app_log("\r\n");
        } else {
          app_log("Failed to create new appkey. Error: 0x%04x\r\n", sc);
        }
      }
      sc = sl_btmesh_prov_scan_unprov_beacons();
      if (sc == SL_STATUS_OK ) {
        app_log("Unprov beacons scan...\r\n");
        state = scanning;
      }else{
        app_log("Failed to initializing unprovisioned beacon scan. Error: 0x%04x", sc);
      }

      sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(100), TIMER_ID_BUTTON_POLL, 0);

      }
      break;

    case sl_btmesh_evt_prov_initialization_failed_id:
      {
        sl_btmesh_evt_prov_initialization_failed_t *initialization_failed_evt = (sl_btmesh_evt_prov_initialization_failed_t *)&(evt->data);
        app_log("Failed:0x%04X\r\n",initialization_failed_evt->result);
      }
      break;

    case sl_btmesh_evt_prov_unprov_beacon_id:
      {
        sl_btmesh_evt_prov_unprov_beacon_t *beacon_evt = (sl_btmesh_evt_prov_unprov_beacon_t *)&(evt->data);
        int i;

        // this example can handle only one bearer type at a time: either PB-GATT or PB-ADV
        #ifdef PROVISION_OVER_GATT
        #define bearer_type 1
        #else
        #define bearer_type 0
        #endif
        if(input_request.bit.prov_start == 0){
          if ((state == scanning) && (beacon_evt->bearer == bearer_type)) {
            app_log("Unprovisioned beacon, UUID: ");

            for(i = 0;i < 16;i++) {
              app_log("%2.2x", beacon_evt->uuid.data[i]);
            }

            // show also the same shortened version that is used on the LCD of switch / light examples
            app_log(" (%2.2x %2.2x) ", beacon_evt->uuid.data[11], beacon_evt->uuid.data[10]);
            app_log("type: %s", beacon_evt->bearer ? "PB-GATT" : "PB-ADV");
            app_log("\r\n");

            memcpy(&_uuid_copy, beacon_evt->uuid.data, 16);

            #ifdef PROVISION_OVER_GATT
            bt_address = beacon_evt->address;
            bt_address_type = beacon_evt->address_type;
            #endif

            app_log("OOB Capabilities: %04x\r\n", beacon_evt->oob_capabilities);

            app_log("-> confirm? (use buttons or keys 'y' / 'n')\r\n");
            // suspend reporting of unprov beacons until user has rejected or accepted this one using buttons PB0 / PB1
            input_request.bit.prov_start = 1;
            sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(100), TIMER_ID_BUTTON_POLL, 0);
          }
        }
      }
      break;

    case sl_btmesh_evt_prov_oob_auth_request_id:
      {
        sl_btmesh_evt_prov_oob_auth_request_t *oob_auth_evt = (sl_btmesh_evt_prov_oob_auth_request_t*)&(evt->data);

        //app_log("Static OOB auth request. Please input OOB(hex)!%d,%d,%d\r\n", oob_auth_evt->output, oob_auth_evt->output_action, oob_auth_evt->output_size);
        input.error = 0;
        input.index = 0;
        sl_bt_system_set_soft_timer(TIMER_MS_2_TIMERTICK(100), TIMER_ID_BUTTON_POLL, 0);
        if(oob_auth_evt->output){
          input_request.bit.oob_blink = 1;
          app_log("Blink. Please input blink number!\r\n");
        }else{
          input_request.bit.oob_static = 1;
          app_log("Static OOB key request. Please input 16 character hexadecimal(0123456789ABCDEF) string!\r\n");
        }
      }
      break;

    case sl_btmesh_evt_prov_oob_display_input_id:
      {
        sl_btmesh_evt_prov_oob_display_input_t *oob_display_evt = (sl_btmesh_evt_prov_oob_display_input_t*)&(evt->data);

        //for(int i = 0; i < oob_display_evt->data.len; i++){
        //    app_log("%02X ", oob_display_evt->data.data[i]);
        //}
        //app_log("\r\n");

        app_log("Please push %d times on your device.\r\n", oob_display_evt->data.data[15]);

      }
      break;

    case sl_btmesh_evt_prov_oob_pkey_request_id:
      {
        app_log("abc\r\n");
      }
      break;

    case sl_btmesh_evt_prov_device_provisioned_id:
      {
      sl_btmesh_evt_prov_device_provisioned_t *prov_evt = (sl_btmesh_evt_prov_device_provisioned_t*)&(evt->data);

      app_log("Node successfully provisioned.\r\nAddress: %4.4x\r\n", prov_evt->address);
      state = provisioned;

      app_log("UUID: 0x");
      for (uint8_t i = 0; i < 16; i++)
        app_log("%02X", prov_evt->uuid.data[i]);
      app_log("\r\n");

      provisionee_address = prov_evt->address;
      /* kick of next phase which is reading DCD from the newly provisioned node */
      trigger_next_state(TIMER_ID_GET_DCD);
      }
      break;

    case sl_btmesh_evt_prov_provisioning_failed_id:
      {
      sl_btmesh_evt_prov_provisioning_failed_t *fail_evt = (sl_btmesh_evt_prov_provisioning_failed_t*)&(evt->data);
      app_log("Provisioning failed. Reason: 0x%02x\r\n", fail_evt->reason);
      state = scanning;
      }
      break;

    case sl_btmesh_evt_prov_provisioning_suspended_id:
      break;
      {
      sl_btmesh_evt_prov_provisioning_suspended_t *suspended_evt = (sl_btmesh_evt_prov_provisioning_suspended_t*)&(evt->data);
      app_log("provisioning suspended. Reason: 0x%02x\r\n", suspended_evt->reason);
      }
      break;
    case sl_btmesh_evt_prov_start_sent_id:
      app_log("provision start sent.\r\n");
      break;
    case sl_btmesh_evt_prov_capabilities_id:
      {
        sl_btmesh_evt_prov_capabilities_t *capabilities_evt = (sl_btmesh_evt_prov_capabilities_t*)&(evt->data);
        app_log("provision capabilities.\r\n"
                                        "elements: 0x%02x\r\n"
                                        "algorithms:0x%04x\r\n"
                                        "pkey_type: 0x%02x\r\n"
                                        "static_oob_type: 0x%02x\r\n"
                                        "ouput_oob_size:0x%04x\r\n"
                                        "output_oob_action: 0x%02x\r\n"
                                        "input_oob_size: 0x%02x\r\n"
                                        "intput_oob_action: 0x%02x\r\n",

                                        capabilities_evt->elements,
                                        capabilities_evt->algorithms,
                                        capabilities_evt->pkey_type,
                                        capabilities_evt->static_oob_type,
                                        capabilities_evt->ouput_oob_size,
                                        capabilities_evt->output_oob_action,
                                        capabilities_evt->input_oob_size,
                                        capabilities_evt->intput_oob_action);

        sc = sl_btmesh_prov_set_device_address(_uuid_copy, 0x2010);
        if (sc != SL_STATUS_OK) {
          app_log("Failed to set address for node. (0x%04X)\r\n", sc);
        }

        sc = sl_btmesh_prov_continue_provisioning(_uuid_copy);
        if (sc == SL_STATUS_OK) {
          app_log("Provision continue ...\r\n");
        } else {
          app_log("Failed call to provision continue. Error: 0x%04X\r\n", sc);
        }
      }
      break;

    case sl_btmesh_evt_config_client_dcd_data_id:
    {
      sl_btmesh_evt_config_client_dcd_data_t *pDCD = (sl_btmesh_evt_config_client_dcd_data_t *)&(evt->data);
      app_log("DCD data event, received %u bytes\r\n", pDCD->data.len);

      // copy the data into one large array. the data may come in multiple smaller pieces.
      // the data is not decoded until all DCD events have been received (see below)
      if((_dcd_raw_len + pDCD->data.len) <= 256)
      {
        memcpy(&(_dcd_raw[_dcd_raw_len]), pDCD->data.data, pDCD->data.len);
        _dcd_raw_len += pDCD->data.len;
      }
    }
      break;

    case sl_btmesh_evt_config_client_dcd_data_end_id:
    {
      if(evt->data.evt_config_client_dcd_data_end.result) {
        app_log("reading DCD failed\r\n");
        // try again:
        config_retry(TIMER_ID_GET_DCD);
      } else {
        app_log("DCD data end event. Decoding the data.\r\n");
        // decode the DCD content
        DCD_decode();

        // check the desired configuration settings depending on what's in the DCD
        config_check();

        // sanity check: make sure there is at least one application key to be bound to a model
        if(_sConfig.num_bind == 0) {
          app_log("ERROR: don't know how to configure this node, no appkey bindings defined\r\n");
        } else {
          // next step : send appkey to device
          trigger_next_state(TIMER_ID_APPKEY_ADD);
        }
      }
    }
      break;

    case sl_btmesh_evt_config_client_appkey_status_id:
      /* This event is created when a response for an add application key or a remove
       * application key request is received, or the request times out.  */
      if(state == waiting_appkey_ack) {
        uint16_t res = evt->data.evt_config_client_appkey_status.result;
        if(!res) {
          app_log("Appkey added\r\n");
          // move to next step which is binding appkey to models
          trigger_next_state(TIMER_ID_APPKEY_BIND);
        } else {
          app_log("Add appkey failed with code 0x%04X\r\n", res);
          // try again:
          config_retry(TIMER_ID_APPKEY_ADD);
        }
      } else {
        app_log("ERROR: unexpected appkey status event? (state = %u)\r\n", state);
      }
    break;

    case sl_btmesh_evt_config_client_binding_status_id:
      /* Status event for binding and unbinding application keys and models. */
      if(state == waiting_bind_ack) {
        uint16_t res = evt->data.evt_config_client_appkey_status.result;
        if(!res) {
          app_log("Bind complete\r\n");
          _sConfig.num_bind_done++;

          if(_sConfig.num_bind_done < _sConfig.num_bind) {
            // more model<->appkey bindings to be done
            trigger_next_state(TIMER_ID_APPKEY_BIND);
          } else {
            // move to next step which is configuring publish settings
            trigger_next_state(TIMER_ID_PUB_SET);
          }
        } else {
          app_log("Appkey bind failed with code 0x%04X\r\n", res);
          // try again:
          config_retry(TIMER_ID_APPKEY_BIND);
        }
      } else {
        app_log("ERROR: unexpected binding status event? (state = %u)\r\n", state);
      }
      break;

    case sl_btmesh_evt_config_client_model_pub_status_id:
      /*Status event for get model publication state, set model publication state, commands. */
      if(state == waiting_pub_ack) {
        uint16_t res = evt->data.evt_config_client_model_pub_status.result;
        if(!res) {
          app_log(" pub set OK\r\n");
          _sConfig.num_pub_done++;

          if(_sConfig.num_pub_done < _sConfig.num_pub) {
            // more publication settings to be done
            trigger_next_state(TIMER_ID_PUB_SET);
          } else {
            // move to next step which is configuring subscription settings
            trigger_next_state(TIMER_ID_SUB_ADD);
          }
        } else {
          app_log("Setting pub failed with code 0x%04X\r\n", res);
          // try again:
          config_retry(TIMER_ID_PUB_SET);
        }
      } else {
        app_log("ERROR: unexpected pub status event? (state = %u)\r\n", state);
      }
      break;

    case sl_btmesh_evt_config_client_model_sub_status_id:
      /* status event for subscription changes */
      if(state == waiting_sub_ack) {
        uint16_t res = evt->data.evt_config_client_model_sub_status.result;
        if(!res) {
          app_log("Sub add OK\r\n");
          _sConfig.num_sub_done++;
          if(_sConfig.num_sub_done < _sConfig.num_sub) {
            // more subscription settings to be done
            trigger_next_state(TIMER_ID_SUB_ADD);
          } else {
            app_log("***\r\nconfiguration complete\r\n***\r\n");
            #ifdef PROVISION_OVER_GATT
            // close connection if provisioning was done over PB-GATT:
              app_log("closing connection...\r\n");
              sl_bt_connection_close(conn_handle);

            // restart scanning of unprov beacons
            sc = sl_btmesh_prov_scan_unprov_beacons();
            if (sc) {
                app_log("Failure initializing unprovisioned beacon scan. Result: 0x%04X\r\n", sc);
            }else{
              state = scanning;
            }
            #else
            state = scanning;
            #endif
          }
        } else {
          app_log("Setting sub failed with code 0x%04X\r\n", res);
          // try again:
          config_retry(TIMER_ID_SUB_ADD);
        }
      } else {
        app_log("ERROR: unexpected sub status event? (state = %u)\r\n", state);
      }
      break;

    default:
      app_log("BTmesh unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt->header, (evt->header >> 16) & 0xFF, (evt->header >> 24) & 0xFF);
      break;
  }
}
