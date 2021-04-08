/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "boards.h"
#include "bsp.h"
#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"
#include "bsp_btn_ble.h"

#include "ble_db_discovery.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_gap_c.h"
#include "ble_ess_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG        1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO       3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define WIND_LVL_2                  700
#define WIND_LVL_3                  800

#define CMD_SET_TEMPERATURE         '1'
#define CMD_SET_HUMIDITY            '2'
#define CMD_SET_VENTILATION         '3'

#define CMD_SET_UPDATE_LIVING_ROOM  '0'
#define CMD_SET_UPDATE_BEDROOM      '1'
#define CMD_SET_UPDATE_KITCHEN      '2'
#define CMD_SET_UPDATE_BATHROOM     '3'
#define CMD_SET_UPDATE_ENTRY_ROOM   '4'

#define CMD_SET_VENT_LOW_OFF        '0'
#define CMD_SET_VENT_LOW_ON         '1'
#define CMD_SET_VENT_MEDIUM_OFF     '2'
#define CMD_SET_VENT_MEDIUM_ON      '3'
#define CMD_SET_VENT_HIGH_OFF       '4'
#define CMD_SET_VENT_HIGH_ON        '5'
#define CMD_SET_VENT_VENT_OFF       '6'
#define CMD_SET_VENT_VENT_ON        '7'
#define CMD_SET_VENT_MANUAL_OFF     '8'
#define CMD_SET_VENT_MANUAL_ON      '9'

#define LIVING_ROOM_CONN_HANDLE     0
#define BEDROOM_CONN_HANDLE         1
#define KITCHEN_CONN_HANDLE         2
#define BATHROOM_CONN_HANDLE        3
#define ENTRY_ROOM_CONN_HANDLE      4
#define WIATER_CONN_HANDLE          5

#define LIVING_ROOM_DEV_NAME

#define TX_PIN_NUMBER               6
#define RX_PIN_NUMBER               8

#define USBD_POWER_DETECTION        true

#define LED_CDC_ACM_OPEN            (BSP_BOARD_LED_3)

#define CDC_ACM_COMM_INTERFACE      0
#define CDC_ACM_COMM_EPIN           NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE      1
#define CDC_ACM_DATA_EPIN           NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT          NRF_DRV_USBD_EPOUT1

#define RX_SIZE                     5

char dev_name[NRF_SDH_BLE_CENTRAL_LINK_COUNT][7];

static uint8_t  wind_value;
static uint8_t  prev_wind_value;
static uint16_t conn_handles[6] = { NRF_SDH_BLE_CENTRAL_LINK_COUNT + 1 };

static bool manual_on = false;
static bool vent_on = false;

static ble_uuid_t const m_ess_uuid =
{
    .uuid = BLE_UUID_ENV_SENSING_SERVICE,
    .type = BLE_UUID_TYPE_BLE
};

static char m_cdc_data_array[RX_SIZE];

static void cdc_acm_user_ev_handler ( app_usbd_class_inst_t const *p_inst,
                                      app_usbd_cdc_acm_user_event_t event );

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_GAP_C_ARRAY_DEF(m_ble_gap_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);                                             
BLE_ESS_C_ARRAY_DEF(m_ble_ess_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Set up the first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_cdc_data_array,
                                                   1);
            bsp_board_led_on(LED_CDC_ACM_OPEN);
            NRF_LOG_INFO("CDC ACM port opened");
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("CDC ACM port closed");
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            static uint8_t index = 0;
            static uint8_t done_flag = 0;
            index++;

            do
            {
                if ((m_cdc_data_array[index - 1] == '\n') ||
                    (m_cdc_data_array[index - 1] == '\r') ||
                    (index >= (RX_SIZE)))
                {
                    index = 0;
                }

                /*Get amount of data transferred*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);


                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            &m_cdc_data_array[index],
                                            1);
                if (ret == NRF_SUCCESS)
                {
                    index++;
                    done_flag = 0;
                }
                else
                {
                    done_flag = 1;
                }
            }
            while (ret == NRF_SUCCESS);

            if (done_flag)
            {
                NRF_LOG_INFO("m_cdc_data_array[0] = %0d", m_cdc_data_array[0]);
                NRF_LOG_INFO("m_cdc_data_array[1] = %0d", m_cdc_data_array[1]);
                
                switch(m_cdc_data_array[0])
                {
                    case CMD_SET_TEMPERATURE:
                        switch(m_cdc_data_array[1])
                        {
                            case CMD_SET_UPDATE_LIVING_ROOM:
                                ble_ess_c_temperature_read(&m_ble_ess_c[conn_handles[LIVING_ROOM_CONN_HANDLE]]);
                                break;
                            case CMD_SET_UPDATE_BEDROOM:
                                ble_ess_c_temperature_read(&m_ble_ess_c[conn_handles[BEDROOM_CONN_HANDLE]]);
                                break;
                            case CMD_SET_UPDATE_KITCHEN:
                                break;
                            case CMD_SET_UPDATE_BATHROOM:
                                break;
                            case CMD_SET_UPDATE_ENTRY_ROOM:
                                break;
                            default:
                                break;
                        }
                        break;
                    case CMD_SET_HUMIDITY:
                        switch(m_cdc_data_array[1])
                        {
                            case CMD_SET_UPDATE_LIVING_ROOM:
                                ble_ess_c_humidity_read(&m_ble_ess_c[conn_handles[LIVING_ROOM_CONN_HANDLE]]);
                                break;
                            case CMD_SET_UPDATE_BEDROOM:
                                ble_ess_c_humidity_read(&m_ble_ess_c[conn_handles[BEDROOM_CONN_HANDLE]]);
                                break;
                            case CMD_SET_UPDATE_KITCHEN:
                                break;
                            case CMD_SET_UPDATE_BATHROOM:
                                break;
                            case CMD_SET_UPDATE_ENTRY_ROOM:
                                break;
                            default:
                                break;
                        }
                        break;
                    case CMD_SET_VENTILATION:
                        switch(m_cdc_data_array[1])
                        {
                            case CMD_SET_VENT_LOW_OFF:
                                wind_value = 0;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                break;
                            case CMD_SET_VENT_LOW_ON:
                                wind_value = 1;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                break;
                            case CMD_SET_VENT_MEDIUM_OFF:
                                wind_value = 0;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                break;
                            case CMD_SET_VENT_MEDIUM_ON:
                                wind_value = 2;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                break;
                            case CMD_SET_VENT_HIGH_OFF:
                                wind_value = 0;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                break;
                            case CMD_SET_VENT_HIGH_ON:
                                wind_value = 3;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                break;
                            case CMD_SET_VENT_VENT_OFF:
                                wind_value = 0;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                vent_on = false;
                                break;
                            case CMD_SET_VENT_VENT_ON:
                                vent_on = true;
                                ble_ess_c_humidity_read(&m_ble_ess_c);
                                break;
                            case CMD_SET_VENT_MANUAL_OFF:
                                wind_value = 0;
                                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                                prev_wind_value = wind_value;
                                manual_on = false;
                                ble_ess_c_humidity_read(&m_ble_ess_c);
                                break;
                            case CMD_SET_VENT_MANUAL_ON:
                                manual_on = true;
                                break;
                            default:
                                break;
                        }
                        break;
                    default:
                        break;
                }

            }

            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;

        case APP_USBD_EVT_DRV_RESUME:
            break;

        case APP_USBD_EVT_STARTED:
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
        {
            NRF_LOG_INFO("USB power removed");
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            app_usbd_stop();
        }
            break;

        case APP_USBD_EVT_POWER_READY:
        {
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
        }
            break;

        default:
            break;
    }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static void ess_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void gap_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected = p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_ess_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_gap_c_on_db_disc_evt(&m_ble_gap_c[p_evt->conn_handle], p_evt);
    ble_ess_c_on_db_disc_evt(&m_ble_ess_c[p_evt->conn_handle], p_evt);
}

static void ble_ess_c_evt_handler(ble_ess_c_t * p_ble_ess_c, ble_ess_c_evt_t const * p_ble_ess_evt)
{
    ret_code_t err_code;
    char str[92];
    size_t size;

    switch (p_ble_ess_evt->evt_type)
    {
        case BLE_ESS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_ess_c_handles_assign(p_ble_ess_c, p_ble_ess_evt->conn_handle, &p_ble_ess_evt->params.handles);
            APP_ERROR_CHECK(err_code);

            if(p_ble_ess_c->handles.ess_humidity_cccd_handle != NULL)
            {
              err_code = ble_ess_c_humidity_read(p_ble_ess_c);
              APP_ERROR_CHECK(err_code);

              err_code = ble_ess_c_humidity_notif_enable(p_ble_ess_c);
              APP_ERROR_CHECK(err_code);
            }
            
            if(p_ble_ess_c->handles.ess_temperature_cccd_handle != NULL)
            {
              err_code = ble_ess_c_temperature_read(p_ble_ess_c);
              APP_ERROR_CHECK(err_code);

              err_code = ble_ess_c_temperature_notif_enable(p_ble_ess_c);
              APP_ERROR_CHECK(err_code);
            }

            NRF_LOG_INFO("Connected to device with ESS.");
            break;

        case BLE_ESS_C_EVT_HUMIDITY_EVT:
            NRF_LOG_INFO("HUMIDITY UPDATED!");
            NRF_LOG_INFO("%0d", p_ble_ess_evt->params.humidity.humidity_value);
            if(!manual_on)
            {
              if(p_ble_ess_evt->params.humidity.humidity_value < WIND_LVL_2)
              {
                wind_value = 1;
              }
              else if (p_ble_ess_evt->params.humidity.humidity_value > WIND_LVL_2 && p_ble_ess_evt->params.humidity.humidity_value < WIND_LVL_3)
              {
                wind_value = 2;
              }
              else if (p_ble_ess_evt->params.humidity.humidity_value > WIND_LVL_3)
              {
                wind_value = 3;
              }
            }

            NRF_LOG_INFO("UPDATING WIND VALUE WITH: 0x%X", wind_value);

            if(conn_handles[WIATER_CONN_HANDLE] < NRF_SDH_BLE_CENTRAL_LINK_COUNT + 1)
            {
              if(prev_wind_value != wind_value)
              {
                ble_ess_c_wind_status_send(&m_ble_ess_c[conn_handles[WIATER_CONN_HANDLE]], wind_value);
                prev_wind_value = wind_value;
                size_t size = sprintf(str,"{\n\"room\": \"%s\",\n\"uuid\": 0x%X,\n\"value\": %3d\n}\n", dev_name[conn_handles[WIATER_CONN_HANDLE]], BLE_UUID_TRUE_WIND_SPEED_CHARACTERISTIC, wind_value);
                app_usbd_cdc_acm_write( &m_app_cdc_acm,
                                        str,
                                        size);
              }
            }
            else
            {
              NRF_LOG_INFO("conn_handles[WIATER_CONN_HANDLE] is NULL");
            }

            size = sprintf(str,"{\n\"room\": \"%s\",\n\"uuid\": 0x%X,\n\"value\": %3d\n}\n", dev_name[p_ble_ess_c->conn_handle], BLE_UUID_HUMIDITY_CHARACTERISTIC, p_ble_ess_evt->params.humidity.humidity_value);
            app_usbd_cdc_acm_write( &m_app_cdc_acm,
                                        str,
                                        size);
            break;

        case BLE_ESS_C_EVT_TEMPERATURE_EVT:
            NRF_LOG_INFO("TEMPERATURE UPDATED!");
            NRF_LOG_INFO("%0d", p_ble_ess_evt->params.temperature.temperature_value);

            size = sprintf(str,"{\n\"room\": \"%s\",\n\"uuid\": 0x%X,\n\"value\": %3d\n}\n", dev_name[p_ble_ess_c->conn_handle], BLE_UUID_TEMPERATURE_CHARACTERISTIC, p_ble_ess_evt->params.temperature.temperature_value);
            app_usbd_cdc_acm_write( &m_app_cdc_acm,
                                    str,
                                    size);
            break;

        case BLE_ESS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}

static void ble_gap_c_evt_handler(ble_gap_c_t * p_ble_gap_c, const ble_gap_c_evt_t * p_ble_gap_evt)
{
  uint32_t err_code;

  switch(p_ble_gap_evt->evt_type)
  {
    case BLE_GAP_C_EVT_DISCOVERY_COMPLETE:
      NRF_LOG_INFO("GAP discovery complete.");
      err_code = ble_gap_c_handles_assign(p_ble_gap_c, p_ble_gap_evt->conn_handle, &p_ble_gap_evt->handle);
      APP_ERROR_CHECK(err_code);

      NRF_LOG_INFO("Reading device name from conn_handle %0d", p_ble_gap_evt->conn_handle);
      err_code = ble_gap_c_periph_dev_name_read(p_ble_gap_c);
      APP_ERROR_CHECK(err_code);

      break;

    case BLE_GAP_C_EVT_DEVICE_NAME_READ:

      for (uint32_t i = 0; i < p_ble_gap_evt->data_len; i++)
      {
        dev_name[p_ble_gap_evt->conn_handle][i] = p_ble_gap_evt->p_data[i];
      }
      NRF_LOG_INFO("Device name: %s, conn_handle %0d", dev_name[p_ble_gap_evt->conn_handle], p_ble_gap_evt->conn_handle);

      if(!strcmp(dev_name[p_ble_gap_evt->conn_handle],"AM2305"))
      {
        conn_handles[BEDROOM_CONN_HANDLE] = p_ble_gap_evt->conn_handle;
      }
      else if(!strcmp(dev_name[p_ble_gap_evt->conn_handle], "ROOM01"))
      {
        conn_handles[LIVING_ROOM_CONN_HANDLE] = p_ble_gap_evt->conn_handle;
      }
      else if(!strcmp(dev_name[p_ble_gap_evt->conn_handle], "Wiater"))
      {
        conn_handles[WIATER_CONN_HANDLE] = p_ble_gap_evt->conn_handle;
      }
      break;

    default:
      break;
  }
}

static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:

            NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.", p_gap_evt->conn_handle);

            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

            err_code = ble_gap_c_handles_assign(&m_ble_gap_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_ess_c_handles_assign(&m_ble_ess_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            if(ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
              err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
              APP_ERROR_CHECK(err_code);
            }
            else
            {
              scan_start();
              err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
              APP_ERROR_CHECK(err_code);
            }

            break;

        case BLE_GAP_EVT_DISCONNECTED:

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);
            scan_start();
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        default:
            break;
    }
}

static void ess_c_init(void)
{
    ret_code_t       err_code;
    ble_ess_c_init_t init;

    init.evt_handler   = ble_ess_c_evt_handler;
    init.error_handler = ess_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
      err_code = ble_ess_c_init(&m_ble_ess_c[i], &init);
      APP_ERROR_CHECK(err_code);
    }
}

static void gap_c_init(void)
{
    ret_code_t       err_code;
    ble_gap_c_init_t init;

    init.evt_handler   = ble_gap_c_evt_handler;
    init.error_handler = gap_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    for(uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
      err_code = ble_gap_c_init(&m_ble_gap_c[i], &init);
      APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}

static void gpio_init(void)
{
  for (int i = 0; i < 3; i++)
  {
    nrf_gpio_cfg_output( i + 29);
    nrf_gpio_pin_clear( i + 29);
  }
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    if (NRF_LOG_PROCESS() == false)
    {
        err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


int main(void)
{
    ret_code_t err_code;
    // Initialize.
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    log_init();
    gpio_init();
    buttons_leds_init();

    app_usbd_serial_num_generate();
    err_code = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(err_code);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    err_code = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(err_code);

    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    gap_c_init();
    ess_c_init();
    ble_conn_state_init();
    scan_init();

    // Start execution.
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();

    err_code = app_usbd_power_events_enable();
    APP_ERROR_CHECK(err_code);

    for (;;)
    {
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
        idle_state_handle();
    }
}