# 2026_radar

## 云台代码逻辑梳理

本代码基于2026_radar结构进行重构
本文档用于快速检索和理解云台控制在 `Task` 与 `Chariot` 层的基本执行架构。

### 1. 整体架构

云台逻辑划分如下：

1. **[Task 层](Task/src)**（系统调度）：轮询与驱动
2. **[ISR 层](Task/src/tsk_isr.cpp)**（中断）：接收底层通信数据并提交给云台
3. **[Chariot 层](Chariot/src/crt_gimbal.cpp)**（控制算法）：状态机、角度规划与闭环执行的实体

---

### 2. Chariot 层：[`Gimbal` 核心逻辑](Chariot/inc/crt_gimbal.hpp)

[`Gimbal`](Chariot/inc/crt_gimbal.hpp) 是云台的执行主体，统管 IMU、双轴电机和遥控器解析。

#### 2.1 初始化

[**`Gimbal::init()`**](Chariot/src/crt_gimbal.cpp#L45)

- 初始化 DWT 计时
- 注册 CAN1 / CAN2 回调
- 注册遥控器串口回调
- 初始化 `IMU` 硬件解算

#### 2.2 控制主循环

[**`Gimbal::controlLoop()`**](Chariot/src/crt_gimbal.cpp#L55)
每毫秒被 Task 周期性调用，按以下时序工作：

1. **[`modeSelect()`](Chariot/src/crt_gimbal.cpp#L91)**：解析遥控器拨杆
   - 离线或右下：`GIMBAL_NO_FORCE`
   - 右中：`MANUAL_CONTROL`
   - 右上：`AUTO_CONTROL`
   - 左上：`GIMBAL_LOCK`
   - (右下时) 左中切上：进入 `CALIBRATION`
2. **[`targetOrientationPlan()`](Chariot/src/crt_gimbal.cpp#L66)**：目标角度规划
   - 模式为 `MANUAL_CONTROL` 时，使用**增量式**改变角度并限幅（即手松开后，保持当前姿态）。
3. **[`pitchControl()`](Chariot/src/crt_gimbal.cpp#L131) / [`yawControl()`](Chariot/src/crt_gimbal.cpp#L154)**：解算并执行电机控制
   - `GIMBAL_NO_FORCE`：开环直接给 0
   - 其他：通过极联控制方法 [**`externalClosedloopControl()`**](Chariot/src/crt_gimbal.cpp#L143)，输入角度误差与角速度。
4. **[`transmitGimbalMotorData()`](Chariot/src/crt_gimbal.cpp#L176)**：统一按 CAN 帧协议进行物理层下发。

#### 2.3 数据解析（底层驱动钩子）

- 遥控器解析：[`receiveRemoteControlDataFromISR()`](Chariot/src/crt_gimbal.cpp#L86)
- 电机反馈解析：[`receiveGimbalMotorDataFromISR()`](Chariot/src/crt_gimbal.cpp#L80)

---

### 3. Task 层实现

#### 3.1 任务调度中心

- **云台主任务**：[**`gimbal_task`**](Task/src/tsk_gimbal.cpp#L70)
  负责初始化硬件对象，绑定 VOFA 调参（关注 pitch PID），以及 `1ms` 级别周期触发 `gimbal.controlLoop()`。
- **IMU 任务**：[**`imu_task`**](Task/src/tsk_imu.cpp#L32)
  独立执行，单周期调用 `gimbal.imuLoop()` 解算 Euler 姿态角。
- **VOFA 调试穿插说明**：代码中分布的大量 `vofa.writeData(...)` 及 `AddParameterListener(...)`（特别是任务里那一大串）仅用于上位机波形监控和在线调参。它们不参与核心控制闭环，阅读主体控制逻辑时请直接忽略这部分代码，以免干扰思路。

#### 3.2 中断转发表

所有中断统一定义于 [**`tsk_isr.cpp`**](Task/src/tsk_isr.cpp)，直接分发给上述 `Gimbal` 对应的接收 Hook：

- [`dr16RxCallback()`](Task/src/tsk_isr.cpp#L30) -> 给遥控器解析
- [`can1RxCallback()`](Task/src/tsk_isr.cpp#L35) -> 给电机解析
- [`can2RxCallback()`](Task/src/tsk_isr.cpp#L40) -> 给电机解析

### 3.Usb protocol

#### Transmit

- send the gimbal status to upper machine:

> `Chariot/src/crt_gimbal.cpp`

```cpp
uint8_t Gimbal::transmitGimbalMotorDataViaUsb()
{
    // header 0
    uint32_t txbufIndex = 0;
    // After the assignment, `txbuf_index` becomes `1`.
    m_usbTxBuf[txbufIndex++] = m_usbTxSOF; // SOF 0x3A, remember to check twice

    // Roll angle (1-4)
    const float rollAngle = m_imu->getEulerAngle().x;
    memcpy(m_usbTxBuf + txbufIndex, &rollAngle, sizeof(float));
    txbufIndex += sizeof(float);

    // Pitch angle of motor (5 - 8)
    const float pitchAngle = -m_imu->getEulerAngle().y;
    memcpy(m_usbTxBuf + txbufIndex, &pitchAngle, sizeof(float)); // rad unit
    txbufIndex += sizeof(float);

    // Yaw angle of motor (9 - 12)
    const float yawAngle = m_imu->getEulerAngle().z;
    memcpy(m_usbTxBuf + txbufIndex, &yawAngle, sizeof(float)); // rad unit
    txbufIndex += sizeof(float);

    // the quaternion of IMU (13 - 28)
    const float *quaternion = m_imu->getQuaternion();
    memcpy(m_usbTxBuf + txbufIndex, quaternion, sizeof(float) * 4);
    txbufIndex += sizeof(float) * 4;

    // end of frame (29)
    m_usbTxBuf[txbufIndex] = m_usbTxEOF; // EOF 0xAA
    txbufIndex += 1;

    return CDC_Transmit_FS(m_usbTxBuf, static_cast<uint16_t>(txbufIndex));
}
```

#### Recv

> `CubeMX_BSP/Inc/usbd_cdc_if.h`

```cpp
// define the data structure of the message received from USB (from InfantryDL)
const static uint8_t USB_RX_MSG_HEADER = 0xA3; // the header of the message from InfantryDL, used for data validation
typedef struct  {
    uint8_t header;  // 0xA3 for InfantryDL
    float pitch;
    float yaw;
    uint8_t found;
    uint8_t shoot_or_not;
} __attribute__((packed)) usbRxMsgT;

extern usbRxMsgT usbRxMsg; // export the msg instance
```

> `CubeMX_BSP/Src/usbd_cdc_if.c`

```cpp
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    // handel the received data, and copy the data to the global variable `usbRxMsg` if the data is valid
    if (Buf[0] == USB_RX_MSG_HEADER && *Len == sizeof(usbRxMsgT)) {
        memcpy(&usbRxMsg, Buf, sizeof(usbRxMsgT)); // copy the data to the global variable
    }
  return (USBD_OK);
  /* USER CODE END 6 */
}
```
