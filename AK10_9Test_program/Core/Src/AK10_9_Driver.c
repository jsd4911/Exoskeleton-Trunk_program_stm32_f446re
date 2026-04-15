#include "AK10_9_Driver.h"

// AK10-9 馬達的物理極限參數 (依據原廠 MIT 模式通訊協定)
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -65.0f
#define T_MAX 65.0f

// 內部轉換函式：將浮點數映射並壓縮為整數 (供 CAN 傳輸)
static int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void AK10_9_EnableMotor(CAN_HandleTypeDef *hcan, uint32_t motor_id) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; // 啟動指令碼

    TxHeader.StdId = motor_id;
    TxHeader.ExtId = 0x00;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
}

void AK10_9_DisableMotor(CAN_HandleTypeDef *hcan, uint32_t motor_id) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}; // 關閉指令碼

    TxHeader.StdId = motor_id;
    TxHeader.ExtId = 0x00;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
}

void AK10_9_SendCommand(CAN_HandleTypeDef *hcan, uint32_t motor_id, float p_des, float v_des, float kp, float kd, float t_ff) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    // 1. 限制並轉換數值
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    // 2. 依據 MIT 協議將資料打包進 8 Bytes
    TxData[0] = p_int >> 8;
    TxData[1] = p_int & 0xFF;
    TxData[2] = v_int >> 4;
    TxData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    TxData[4] = kp_int & 0xFF;
    TxData[5] = kd_int >> 4;
    TxData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    TxData[7] = t_int & 0xFF;

    TxHeader.StdId = motor_id;
    TxHeader.ExtId = 0x00;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    // 3. 確認發送信箱有空位才發送 (防止 ORE 塞車當機)
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
    }
}
