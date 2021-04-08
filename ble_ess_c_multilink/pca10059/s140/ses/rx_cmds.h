#include <stdint.h>
#include <stdbool.h>
#include "ble_ess_c.h"

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

void decode_rx_cmd(char rx_cmd[]);
