/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : 测试任务
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "crt_gimbal.hpp"

/* Define --------------------------------------------------------------------*/
/******************************************************************************
 *                            电机相关
 ******************************************************************************/
/* PID -----------------------------------------*/
// Yaw
CascadePID::PIDParam yawOuterParam = {
    YAW_OUTER_KP,        // Kp
    YAW_OUTER_KI,        // Ki
    YAW_OUTER_KD,        // Kd
    YAW_OUTER_OUT_LIMIT, // outputLimit
    YAW_OUTER_IOUT_LIMIT // intergralLimit
};
CascadePID::PIDParam yawInnerParam = {
    YAW_INNER_KP,        // Kp
    YAW_INNER_KI,        // Ki
    YAW_INNER_KD,        // Kd
    YAW_INNER_OUT_LIMIT, // outputLimit
    YAW_INNER_IOUT_LIMIT // intergralLimit
};
LowPassFilter<fp32> yawInnerLPF(YAW_INNER_LOWPASS_FILTER_PARA);
CascadePID yawPID(yawOuterParam, yawInnerParam, nullptr, &yawInnerLPF);
// Pitch
CascadePID::PIDParam pitchOuterParam = {
    PITCH_OUTER_KP,        // Kp
    PITCH_OUTER_KI,        // Ki
    PITCH_OUTER_KD,        // Kd
    PITCH_OUTER_OUT_LIMIT, // outputLimit
    PITCH_OUTER_IOUT_LIMIT // intergralLimit
};
CascadePID::PIDParam pitchInnerParam = {
    PITCH_INNER_KP,        // Kp
    PITCH_INNER_KI,        // Ki
    PITCH_INNER_KD,        // Kd
    PITCH_INNER_OUT_LIMIT, // outputLimit
    PITCH_INNER_IOUT_LIMIT // intergralLimit
};
LowPassFilter<fp32> pitchInnerLPF(PITCH_INNER_LOWPASS_FILTER_PARA);
CascadePID pitchPID(pitchOuterParam, pitchInnerParam, nullptr, &pitchInnerLPF);

MotorDM4310 yawMotor(2, 4, 3.141593f, 10, 5, &yawPID);
MotorDM4310 pitchMotor(1, 3, 3.141593f, 10, 5, &pitchPID);

Vofa<12> vofa;



/******************************************************************************
 *                            IMU相关
 ******************************************************************************/
// AHRS算法
Mahony ahrs(AHRS_AUTO_FREQ, AHRS_DEFAULT_FILTER, MAHONY_KP, MAHONY_KI);
// IMU校准数据
BMI088::CalibrationInfo cali = {
    {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z},    // gyroOffset
    {ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z}, // accelOffset
    {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z},       // magnetOffset
    {INSTALL_SPIN_MATRIX}                             // installSpinMatrix
};
// IMU类定义
BMI088 imu(&ahrs, {&hspi1, GPIOA, GPIO_PIN_4}, {&hspi1, GPIOB, GPIO_PIN_0}, cali);

// gimbal
Gimbal gimbal(&yawMotor, &pitchMotor, &imu);

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

extern "C" void gimbal_task(void *argument)
{
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    gimbal.init();
            vofa.AddParameterListener("IP", [](fp32 *newdata) { 
            pitchInnerParam.Kp = *newdata;
            pitchPID.getInnerLoop().pidSetParam(pitchInnerParam);

        });
        //补全PID INNER ORTER
        vofa.AddParameterListener("II", [](fp32 *newdata) { 
            pitchInnerParam.Ki = *newdata;
            pitchPID.getInnerLoop().pidSetParam(pitchInnerParam);
        });
        vofa.AddParameterListener("ID", [](fp32 *newdata) { 
            pitchInnerParam.Kd = *newdata;
            pitchPID.getInnerLoop().pidSetParam(pitchInnerParam);
        });
        vofa.AddParameterListener("IOUT", [](fp32 *newdata) { 
            pitchInnerParam.outputLimit = *newdata;
            pitchPID.getInnerLoop().pidSetParam(pitchInnerParam);
        });
        vofa.AddParameterListener("IINT", [](fp32 *newdata) { 
            pitchInnerParam.intergralLimit = *newdata;
            pitchPID.getInnerLoop().pidSetParam(pitchInnerParam);
        });
        vofa.AddParameterListener("OP", [](fp32 *newdata) { 
            pitchOuterParam.Kp = *newdata;
            pitchPID.getOuterLoop().pidSetParam(pitchOuterParam);
        });
        vofa.AddParameterListener("OI", [](fp32 *newdata) { 
            pitchOuterParam.Ki = *newdata;
            pitchPID.getOuterLoop().pidSetParam(pitchOuterParam);
        });
        vofa.AddParameterListener("OD", [](fp32 *newdata) { 
            pitchOuterParam.Kd = *newdata;
            pitchPID.getOuterLoop().pidSetParam(pitchOuterParam);
        });
        vofa.AddParameterListener("OOUT", [](fp32 *newdata) { 
            pitchOuterParam.outputLimit = *newdata;
            pitchPID.getOuterLoop().pidSetParam(pitchOuterParam);
        });
        vofa.AddParameterListener("OINT", [](fp32 *newdata) { 
            pitchOuterParam.intergralLimit = *newdata;
            pitchPID.getOuterLoop().pidSetParam(pitchOuterParam);
        });
    pitchMotor.setControllerOutputPolarity(true); // 设置Pitch电机控制器输出极性为正（根据实际情况调整）
    while (1) {
        gimbal.controlLoop();
        vofa.writeData(pitchPID.getOuterLoop().pidGetData().output);
        vofa.writeData(pitchPID.getInnerLoop().pidGetData().output);
        vofa.writeData(pitchMotor.getCurrentAngle());
        vofa.writeData(pitchPID.getOuterLoop().pidGetData().pOut);
        vofa.writeData(pitchPID.getOuterLoop().pidGetData().iOut);
        vofa.writeData(pitchPID.getOuterLoop().pidGetData().dOut);
        vofa.writeData(pitchPID.getInnerLoop().pidGetData().pOut);
        vofa.writeData(pitchPID.getInnerLoop().pidGetData().iOut);
        vofa.writeData(pitchPID.getInnerLoop().pidGetData().dOut);


        vofa.sendFrame();
        vTaskDelayUntil(&taskLastWakeTime, 1); // 确保任务以定周期1ms运行
    }
}
