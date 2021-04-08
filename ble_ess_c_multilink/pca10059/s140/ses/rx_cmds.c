#include "rx_cmds.h"
#include "ble_ess_c.h"

void decode_rx_cmd(char m_cdc_data_array[]) 
{
  switch (m_cdc_data_array[0]) 
  {
    case CMD_SET_TEMPERATURE:
      switch (m_cdc_data_array[1]) 
      {
        case CMD_SET_UPDATE_LIVING_ROOM:
          ble_ess_c_temperature_read(&m_ble_ess_c);
          break;
        case CMD_SET_UPDATE_BEDROOM:
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
      switch (m_cdc_data_array[1]) 
      {
        case CMD_SET_UPDATE_LIVING_ROOM:
          ble_ess_c_humidity_read(&m_ble_ess_c);
          break;
        case CMD_SET_UPDATE_BEDROOM:
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
      switch (m_cdc_data_array[1]) 
      {
        case CMD_SET_VENT_LOW_OFF:
          wind_value = 0;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
          prev_wind_value = wind_value;
          break;
        case CMD_SET_VENT_LOW_ON:
          wind_value = 1;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
          prev_wind_value = wind_value;
          break;
        case CMD_SET_VENT_MEDIUM_OFF:
          wind_value = 0;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
          prev_wind_value = wind_value;
          break;
        case CMD_SET_VENT_MEDIUM_ON:
          wind_value = 2;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
          prev_wind_value = wind_value;
          break;
        case CMD_SET_VENT_HIGH_OFF:
          wind_value = 0;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
          prev_wind_value = wind_value;
          break;
        case CMD_SET_VENT_HIGH_ON:
          wind_value = 3;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
          prev_wind_value = wind_value;
          break;
        case CMD_SET_VENT_VENT_OFF:
          wind_value = 0;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
          prev_wind_value = wind_value;
          vent_on = false;
          break;
        case CMD_SET_VENT_VENT_ON:
          vent_on = true;
          ble_ess_c_humidity_read(&m_ble_ess_c);
          break;
        case CMD_SET_VENT_MANUAL_OFF:
          wind_value = 0;
          ble_ess_c_wind_status_send(&m_ble_ess_c[wind_conn_handle], wind_value);
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