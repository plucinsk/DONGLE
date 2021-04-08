/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
 
#ifndef ENV_SENSING_SERVICE_H__
#define ENV_SENSING_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"

#define BLE_ESS_BLE_OBSERVER_PRIO 2
#define BLE_ESS_DEF(_name)                                                                          \
static ble_ess_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_ESS_BLE_OBSERVER_PRIO,                                                     \
                     ble_env_sensing_service_on_ble_evt, &_name)

#define BLE_UUID_ENV_SENSING_SERVICE            0x181A
#define BLE_UUID_TRUE_WIND_SPEED_CHARACTERISTIC 0x2A70

typedef struct ble_ess_c ble_ess_t;

typedef void (*ble_ess_wind_write_handler_t) (uint16_t conn_handle, ble_ess_t * p_ess, uint8_t new_state);

typedef struct
{
  ble_ess_wind_write_handler_t wind_write_handler;
} ble_ess_init_t;

struct ble_ess_c
{
    uint16_t                      conn_handle;
    uint16_t                      service_handle; 
    ble_gatts_char_handles_t      wind_speed_handles;
    ble_ess_wind_write_handler_t  wind_write_handler;
};

void ble_env_sensing_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

uint32_t env_sensing_service_init(ble_ess_t * p_env_sensing_service, const ble_ess_init_t * p_ess_init);

#endif  /* _ ENV_SENSING_SERVICE_H__ */
