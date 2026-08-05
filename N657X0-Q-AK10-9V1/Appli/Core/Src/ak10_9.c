#include "ak10_9.h"


/* 將數值限制在指定範圍內 */
static float AK10_9_Clamp(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


/* 將浮點數轉成 MIT Mode 使用的無號整數 */
static uint32_t AK10_9_FloatToUint(
    float value,
    float minimum,
    float maximum,
    uint32_t bits)
{
    const float span = maximum - minimum;
    const uint32_t full_scale = (1UL << bits) - 1UL;

    const float limited_value =
        AK10_9_Clamp(value, minimum, maximum);

    /*
     * 保留原本已由 F446RE 實機驗證程式使用的截斷方式。
     */
    return (uint32_t)(
        (limited_value - minimum)
        * (float)full_scale
        / span);
}


/* 將 MIT Mode 的無號整數還原成浮點數 */
static float AK10_9_UintToFloat(
    uint32_t value,
    float minimum,
    float maximum,
    uint32_t bits)
{
    const float span = maximum - minimum;
    const uint32_t full_scale = (1UL << bits) - 1UL;

    return ((float)value * span / (float)full_scale)
           + minimum;
}


/* 發送一個 Standard ID、Classic CAN、8-byte 資料幀 */
static HAL_StatusTypeDef AK10_9_SendFrame(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id,
    const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx_header = {0};

    if ((hfdcan == NULL)
        || (data == NULL)
        || (motor_id > 0x7FFU))
    {
        return HAL_ERROR;
    }

    /* 確認 Tx FIFO 還有空間 */
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U)
    {
        return HAL_BUSY;
    }

    tx_header.Identifier = motor_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(
        hfdcan,
        &tx_header,
        data);
}


/* 發送 Enable、Disable 或 Set Zero 特殊命令 */
static HAL_StatusTypeDef AK10_9_SendSpecialCommand(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id,
    uint8_t command)
{
    const uint8_t data[8] =
    {
        0xFFU,
        0xFFU,
        0xFFU,
        0xFFU,
        0xFFU,
        0xFFU,
        0xFFU,
        command
    };

    return AK10_9_SendFrame(
        hfdcan,
        motor_id,
        data);
}


/* 進入 MIT 運轉模式 */
HAL_StatusTypeDef AK10_9_Enable(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id)
{
    return AK10_9_SendSpecialCommand(
        hfdcan,
        motor_id,
        0xFCU);
}


/* 離開 MIT 運轉模式 */
HAL_StatusTypeDef AK10_9_Disable(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id)
{
    return AK10_9_SendSpecialCommand(
        hfdcan,
        motor_id,
        0xFDU);
}


/* 將目前位置設定為零點 */
HAL_StatusTypeDef AK10_9_SetZero(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id)
{
    return AK10_9_SendSpecialCommand(
        hfdcan,
        motor_id,
        0xFEU);
}


/* 發送 MIT Mode 控制命令 */
HAL_StatusTypeDef AK10_9_SendMitCommand(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id,
    float position_rad,
    float velocity_rad_s,
    float kp,
    float kd,
    float torque_ff_nm)
{
    uint8_t data[8];

    const uint32_t position =
        AK10_9_FloatToUint(
            position_rad,
            AK10_9_POSITION_MIN_RAD,
            AK10_9_POSITION_MAX_RAD,
            16U);

    const uint32_t velocity =
        AK10_9_FloatToUint(
            velocity_rad_s,
            AK10_9_VELOCITY_MIN_RAD_S,
            AK10_9_VELOCITY_MAX_RAD_S,
            12U);

    const uint32_t kp_value =
        AK10_9_FloatToUint(
            kp,
            AK10_9_KP_MIN,
            AK10_9_KP_MAX,
            12U);

    const uint32_t kd_value =
        AK10_9_FloatToUint(
            kd,
            AK10_9_KD_MIN,
            AK10_9_KD_MAX,
            12U);

    const uint32_t torque =
        AK10_9_FloatToUint(
            torque_ff_nm,
            AK10_9_TORQUE_MIN_NM,
            AK10_9_TORQUE_MAX_NM,
            12U);

    /*
     * MIT Mode 8-byte 封包：
     *
     * Byte 0～1：Position，16 bits
     * Byte 2～3：Velocity，12 bits
     * Byte 3～4：Kp，12 bits
     * Byte 5～6：Kd，12 bits
     * Byte 6～7：Torque，12 bits
     */
    data[0] = (uint8_t)(position >> 8U);
    data[1] = (uint8_t)(position & 0xFFU);

    data[2] = (uint8_t)(velocity >> 4U);

    data[3] = (uint8_t)(
        ((velocity & 0x0FU) << 4U)
        | (kp_value >> 8U));

    data[4] = (uint8_t)(kp_value & 0xFFU);

    data[5] = (uint8_t)(kd_value >> 4U);

    data[6] = (uint8_t)(
        ((kd_value & 0x0FU) << 4U)
        | (torque >> 8U));

    data[7] = (uint8_t)(torque & 0xFFU);

    return AK10_9_SendFrame(
        hfdcan,
        motor_id,
        data);
}


/* 解析 AK10-9 回傳資料 */
bool AK10_9_ParseFeedback(
    const uint8_t *data,
    uint32_t data_length,
    uint8_t expected_motor_id,
    AK10_9_FeedbackTypeDef *feedback)
{
    uint32_t position;
    uint32_t velocity;
    uint32_t torque;

    if ((data == NULL)
        || (feedback == NULL)
        || (data_length < 6U))
    {
        return false;
    }

    /*
     * AK10-9 MIT 回傳資料的第一個 Byte 為 Motor ID。
     */
    if (data[0] != expected_motor_id)
    {
        return false;
    }

    position =
        ((uint32_t)data[1] << 8U)
        | (uint32_t)data[2];

    velocity =
        ((uint32_t)data[3] << 4U)
        | ((uint32_t)data[4] >> 4U);

    torque =
        (((uint32_t)data[4] & 0x0FU) << 8U)
        | (uint32_t)data[5];

    feedback->motor_id = data[0];

    feedback->position_rad =
        AK10_9_UintToFloat(
            position,
            AK10_9_POSITION_MIN_RAD,
            AK10_9_POSITION_MAX_RAD,
            16U);

    feedback->velocity_rad_s =
        AK10_9_UintToFloat(
            velocity,
            AK10_9_VELOCITY_MIN_RAD_S,
            AK10_9_VELOCITY_MAX_RAD_S,
            12U);

    feedback->torque_nm =
        AK10_9_UintToFloat(
            torque,
            AK10_9_TORQUE_MIN_NM,
            AK10_9_TORQUE_MAX_NM,
            12U);

    /*
     * 部分版本回傳 8 Bytes：
     * Byte 6 = 溫度
     * Byte 7 = 錯誤碼
     *
     * 舊版可能只回傳前 6 Bytes。
     */
    if (data_length >= 8U)
    {
        feedback->has_temperature_and_error = true;
        feedback->temperature_c = data[6];
        feedback->error_code = data[7];
    }
    else
    {
        feedback->has_temperature_and_error = false;
        feedback->temperature_c = 0U;
        feedback->error_code = 0U;
    }

    return true;
}
