#ifndef AK10_9_DRIVER_H
#define AK10_9_DRIVER_H

#include "stm32f4xx_hal.h"

// 啟動馬達 (進入運轉模式)
void AK10_9_EnableMotor(CAN_HandleTypeDef *hcan, uint32_t motor_id);

// 關閉馬達 (進入失能放鬆模式)
void AK10_9_DisableMotor(CAN_HandleTypeDef *hcan, uint32_t motor_id);

// 設定當前位置為硬體零點
void AK10_9_SetZero(CAN_HandleTypeDef *hcan, uint32_t motor_id);

// 發送控制指令 (MIT 控制模式)
// 參數: CAN控制卡, 馬達ID, 目標位置(Rad), 目標速度(Rad/s), 位置剛性(Kp), 速度阻尼(Kd), 前饋扭矩(Nm)
void AK10_9_SendCommand(CAN_HandleTypeDef *hcan, uint32_t motor_id, float p_des, float v_des, float kp, float kd, float t_ff);

#endif
