/*
 * STM32G4 FDCAN1 (CAN 2.0 classic 500kbps)
 */
#include "hal/hal_can.h"
#include "fdcan.h"

static hal_can_rx_cb_t _rx_cb;

static FDCAN_TxHeaderTypeDef   _tx_hdr;
static FDCAN_RxHeaderTypeDef   _rx_hdr;
static uint8_t                 _rx_data[8];

static void _init(uint32_t baud)
{
    (void)baud;
    _rx_cb = NULL;

    FDCAN_FilterTypeDef sf = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0,
        .FilterID2    = 0xFFFF,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sf);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

static void _send(uint32_t id, uint8_t *data, uint8_t len)
{
    _tx_hdr.Identifier       = id;
    _tx_hdr.IdType           = FDCAN_STANDARD_ID;
    _tx_hdr.TxFrameType      = FDCAN_DATA_FRAME;
    _tx_hdr.DataLength       = (len > 8) ? 8 : ((len < 1) ? 1 : len);
    _tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    _tx_hdr.BitRateSwitch    = FDCAN_BRS_OFF;
    _tx_hdr.FDFormat         = FDCAN_CLASSIC_CAN;
    _tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    _tx_hdr.MessageMarker    = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &_tx_hdr, data);
}

static void _set_rx_cb(hal_can_rx_cb_t cb) { _rx_cb = cb; }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (_rx_cb && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &_rx_hdr, _rx_data) == HAL_OK)
            _rx_cb(_rx_hdr.Identifier, _rx_data, _rx_hdr.DataLength);
    }
}

const hal_can_t g_hal_can = {
    .init      = _init,
    .send      = _send,
    .set_rx_cb = _set_rx_cb,
};
