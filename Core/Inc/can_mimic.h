#pragma once

#include "stm32g4xx_hal.h"

#define NUM_PCAN_MESSAGES 7
#define NUM_CCAN_MESAGES 1

#define Information_ID 0x6B0
#define Information_1_ID 0x6B1
#define Information_2_ID 0x6B2
#define Information_3_ID 0x6B3
#define Information_4_ID 0x6B4
#define Charger_Unit1_ID 0x618
#define Charger_Unit2_ID 0x718

#define INFORMATION_LENGTH 6
#define INFORMATION_1_LENGTH 6
#define INFORMATION_2_LENGTH 2
#define INFORMATION_3_LENGTH 6
#define INFORMATION_4_LENGTH 6
#define CHARGER_UNIT1_LENGTH 7
#define CHARGER_UNIT2_LENGTH 7

typedef union {
    uint8_t raw;
    struct {
        uint8_t discharge_relay_enabled : 1;
        uint8_t charge_relay_enabled : 1;
        uint8_t charger_safety_enabled : 1;
        uint8_t malfunction_indicator_active : 1;
        uint8_t multipurpose_input_signal_status : 1;
        uint8_t always_on_signal_status : 1;
        uint8_t is_ready_signal_status : 1;
        uint8_t is_charging_signal_status : 1;
    };
} relay_states_t;

typedef union {
    uint8_t raw[6];
    struct {
        uint16_t pack_current : 16;
        uint16_t pack_voltage : 16;
        uint8_t pack_soc : 8;
        relay_states_t relay_states;
    };
} Information_t;

typedef union {
    uint8_t raw[6];
    struct {
        uint16_t pack_DCL : 16;
        uint16_t pack_CCL : 16;
        uint16_t internal_temp : 16;
    };
} Information_1_t;

typedef union {
    uint8_t raw[2];
    struct {
        uint16_t avg_cell_resistance : 16;
    };
} Information_2_t;

typedef union {
    uint8_t raw[6];
    struct {
        uint16_t max_pack_voltage : 16;
        uint16_t high_cell_voltage : 16;
        uint16_t low_cell_voltage : 16;
    };
} Information_3_t;

typedef union {
    uint8_t raw[6];
    struct {
        uint16_t high_cell_temp : 16;
        uint8_t high_thermistor_id : 8;
        uint16_t low_cell_temp : 16;
        uint8_t low_thermistor_id : 8;
    };
} Information_4_t;

typedef union {
    uint8_t raw[7];
    struct {
        uint8_t chrgCtl_canEnable : 8;
        uint16_t chrgCtl_iacmMaxSet : 16;
        uint16_t chrgCtl_vOutMaxSet : 16;
        uint16_t chrgCtl_iOutMaxSet : 16;
    };
} Charger_Unit1_t;

typedef union {
    uint8_t raw[7];
    struct {
        uint8_t chrgCtl_canEnable : 8;
        uint16_t chrgCtl_iacmMaxSet : 16;
        uint16_t chrgCtl_vOutMaxSet : 16;
        uint16_t chrgCtl_iOutMaxSet : 16;
    };
} Charger_Unit2_t;

void sendPCAN(FDCAN_HandleTypeDef *pcan, uint8_t **data);
void sendCCAN(FDCAN_HandleTypeDef *ccan, uint8_t **data);
HAL_StatusTypeDef FDCAN_SendMessage(FDCAN_HandleTypeDef *whichCAN, uint32_t id, uint8_t *data, uint8_t length);