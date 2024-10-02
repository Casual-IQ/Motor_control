#include "bsp_can.h"
#include "main.h"

#define CAN_6020_M1_ID 0x206

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

moto_info_t motor_info;
uint16_t can_cnt;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->rotor_angle  = (uint16_t)((data)[0] << 8 | (data)[1]);   \
        (ptr)->rotor_speed = (uint16_t)((data)[2] << 8 | (data)[3]);    \
        (ptr)->torque_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temp  = (data)[6];                                       \
    }

/** 
 * @brief CAN filter init
 * @retval none
 */
void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief CAN receive callback
 * @param hcan: CAN handle
 * @retval none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_6020_M1_ID:
        {
            get_motor_measure(&motor_info, rx_data);
            break;
        }

        default:
        {
            break;
        }
    }
}

/**
 * @brief Set GM6020 motor voltage
 * @param hcan: CAN handle
 * @param v1: motor voltage
 * @retval none
 */

void set_GM6020_motor_voltage(CAN_HandleTypeDef* hcan, int16_t v1)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    
    tx_header.StdId = 0x1ff;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 0x08;

    tx_data[0] = 0x00;
    tx_data[1] = 0x00;
    tx_data[2] = (v1>>8);
    tx_data[3] = (v1);

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
