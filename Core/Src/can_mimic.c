#include "can_mimic.h"

const int PCAN_IDs[NUM_PCAN_MESSAGES] = {0x6B0, 0x6B1, 0x6B2, 0x6B3, 0x6B4, 0x618, 0x718};
const int PCAN_LENGTHS[NUM_PCAN_MESSAGES] = {6, 6, 2, 6, 6, 7, 7};
const int CCAN_IDs[NUM_CCAN_MESAGES] = {0x126};
const int CCAN_LENGTHS[NUM_CCAN_MESAGES] = {8};

/**
 * @brief Send the current PCAN data through CAN
 *
 * @param pcan The CAN device for PCAN
 * @param data An array of arrays for the data to send, where the lengths of the
 * inner arrays correspond to the lengths in the PCAN_LENGTHS variable, and the
 * total number of data points is equal to NUM_PCAN_MESSAGES
 */
void sendPCAN(FDCAN_HandleTypeDef *pcan, uint8_t **data) {
    for (int i = 0; i < NUM_PCAN_MESSAGES; i++) {
        FDCAN_SendMessage(pcan, PCAN_IDs[i], data[i], PCAN_LENGTHS[i]);
    }
}

/**
 * @brief Send the current CCAN data through CAN
 *
 * @param ccan The CAN device for CCAN
 * @param data  An array of arrays for the data to send, where the lengths of the
 * inner arrays correspond to the lengths in the CCAN_LENGTHS variable, and the
 * total number of data points is equal to NUM_CCAN_MESSAGES
 */
void sendCCAN(FDCAN_HandleTypeDef *ccan, uint8_t **data) {
    for (int i = 0; i < NUM_CCAN_MESAGES; i++) {
        FDCAN_SendMessage(ccan, CCAN_IDs[i], data[i], CCAN_LENGTHS[i]);
    }
}

HAL_StatusTypeDef FDCAN_SendMessage(FDCAN_HandleTypeDef *whichCAN, uint32_t id, uint8_t *data, uint8_t length) {
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // Always use 8 bytes for CAN 2.0 compatibility
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(whichCAN, &TxHeader, data);
}