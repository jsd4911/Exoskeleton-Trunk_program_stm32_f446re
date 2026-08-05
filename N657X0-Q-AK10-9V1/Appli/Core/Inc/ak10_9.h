#ifndef AK10_9_H
#define AK10_9_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32n6xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* AK10-9 KV100 V1.1：MIT Mode 馬達 ID */
#define AK10_9_DEFAULT_MOTOR_ID      0x01U

/* MIT Mode 編碼範圍 */
#define AK10_9_POSITION_MIN_RAD      (-12.5f)
#define AK10_9_POSITION_MAX_RAD      ( 12.5f)

#define AK10_9_VELOCITY_MIN_RAD_S    (-50.0f)
#define AK10_9_VELOCITY_MAX_RAD_S    ( 50.0f)

#define AK10_9_KP_MIN                (  0.0f)
#define AK10_9_KP_MAX                (500.0f)

#define AK10_9_KD_MIN                (  0.0f)
#define AK10_9_KD_MAX                (  5.0f)

#define AK10_9_TORQUE_MIN_NM         (-65.0f)
#define AK10_9_TORQUE_MAX_NM         ( 65.0f)

/* AK10-9 回傳資料 */
typedef struct
{
    uint8_t motor_id;

    float position_rad;
    float velocity_rad_s;
    float torque_nm;

    uint8_t temperature_c;
    uint8_t error_code;

    bool has_temperature_and_error;

} AK10_9_FeedbackTypeDef;


/* 進入 MIT 運轉模式 */
HAL_StatusTypeDef AK10_9_Enable(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id);


/* 離開 MIT 運轉模式，使馬達放鬆 */
HAL_StatusTypeDef AK10_9_Disable(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id);


/* 將馬達目前位置設為零點 */
HAL_StatusTypeDef AK10_9_SetZero(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id);


/* 發送 MIT 控制命令 */
HAL_StatusTypeDef AK10_9_SendMitCommand(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t motor_id,
    float position_rad,
    float velocity_rad_s,
    float kp,
    float kd,
    float torque_ff_nm);


/* 解析馬達回傳的 6-byte 或 8-byte MIT 資料 */
bool AK10_9_ParseFeedback(
    const uint8_t *data,
    uint32_t data_length,
    uint8_t expected_motor_id,
    AK10_9_FeedbackTypeDef *feedback);


#ifdef __cplusplus
}
#endif

#endif /* AK10_9_H */
