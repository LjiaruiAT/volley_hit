# 🏐 volley_hit — 麦克纳姆轮底盘 + 气缸击球 + JY61 陀螺仪打滑补偿

> STM32F407VGTx · FreeRTOS · CAN · 三全向轮 · 自适应 IMU 打滑补偿 · 气缸驱动

---

## 📋 功能概览

本项目是一个**排球竞赛机器人**主控固件。三麦克纳姆轮全向底盘搭配气缸击球机构，最大的技术亮点是**基于 JY61 陀螺仪的实时打滑检测与自适应补偿**——让机器人在高速运动和碰撞中保持直线行走。

| 子系统 | 硬件 | 通信 | 控制方式 |
|--------|------|------|----------|
| 底盘 (3 麦克纳姆轮) | DJI M3508 ×3 | CAN2 | PID 速度闭环 |
| 击球机构 | 气缸 | GPIO | 时序控制 |
| 姿态感知 | JY61 陀螺仪 | UART4 (DMA) | 三阶低通滤波 + PID |
| 遥控 | 自定义 UART 协议 | UART5 (DMA) | 事件驱动回调 |
| IMU 辅助 | JY61 加速度计 | UART4 | 加速度反馈 |

---

## 🏗️ 架构分层

```
┌───────────────────────────────────────────────┐
│         Task_Init.c（业务逻辑 + 状态机）         │
│  Remote() · Hit_Task() · Remote_JY61()        │
│  Remote_Analysis_Task()                       │
├───────────────────────────────────────────────┤
│  PID2 · PID (位置式) · 斜坡函数 (RampAccel)     │
│  JY61 滤波链 · 打滑补偿 PID                     │
├───────────────────────────────────────────────┤
│  motor · motorEx · CANDrive · JY61            │
├───────────────────────────────────────────────┤
│  comm (遥控协议栈) · My_list · crc_ccitt        │
├───────────────────────────────────────────────┤
│  RMLibHead (跨平台适配层)                       │
├───────────────────────────────────────────────┤
│    FreeRTOS + CMSIS-RTOS2 + STM32 HAL          │
└───────────────────────────────────────────────┘
```

---

## 🧠 核心巧思

### 1. IMU 陀螺仪打滑检测与补偿（最核心亮点）

这是整个项目最具工程价值的模块。全向轮在高速运动、碰撞、地面不平整时会产生滑移，导致机器人自旋。本模块用 JY61 陀螺仪的 Z 轴数据实时检测并补偿滑移。

**数据流：**

```
原始陀螺Z → [低通滤波] → 减去期望Yaw → 得到滑移量
                                         ↓
滑移量 → [低通滤波] → [死区] → 判断打滑标志
                                         ↓
滑移量 → [自适应PID] → [死区] → [低通滤波] → Wz_correction
                                                    ↓
                                         按抓地力比例分配到三车轮
```

**精妙之处：**

**(a) 三阶低通滤波链**

`gyro_z_filt` → `slip_filt` → `corr_filt`，每一级独立调参（`GYRO_LPF_ALPHA` / `SLIP_LPF_ALPHA` / `CORR_LPF_ALPHA`），解决了单级滤波的延迟-平滑矛盾：陀螺需要轻度滤波保实时性，滑移量需要中度滤波滤抖动，最终校正量需要重度滤波防震荡。

**(b) 自适应 PID 增益切换**

```c
if (slip_flag) {
    Kp = 1.3f, Kd = 0.5f, Ki = 0.0003f;   // 打滑时：激进纠正
} else {
    Kp = 0.8f, Kd = 0.35f, Ki = 0.0008f;  // 正常时：保守防抖
}
```

检测到打滑自动切换到高增益模式快速纠偏；恢复正常后切回低增益避免自激震荡。这个切换条件由 `slip > SLIP_THRESHOLD` 驱动，阈值可配。

**(c) 抓地力加权分配**

校正量不是均匀分给三个轮子，而是按抓地力比例分配：

```c
float total_grip = WHEEL1_GRIP_RATIO + WHEEL2_GRIP_RATIO + WHEEL3_GRIP_RATIO;
wheel1_cmd += slip_rpm * (WHEEL1_GRIP_RATIO / total_grip);  // ~39%
wheel2_cmd -= slip_rpm * (WHEEL2_GRIP_RATIO / total_grip);  // ~39%
wheel3_cmd += slip_rpm * (WHEEL3_GRIP_RATIO / total_grip);  // ~22%
```

受力更大的轮子承担更多校正任务——这不是拍脑袋的均分，而是符合物理约束的合理分配。

**(d) 多重死区级联**

`GYRO_DEADZONE` → `SLIP_DEADZONE` → `CORR_OUT_DEADZONE`，层层过滤噪声。静止时陀螺微小的零漂会被三级死区全部吞掉，轮子纹丝不动。

**(e) 积分清零**

校正输出进入死区时主动清零积分项 `JY61_adjust.error_inter = 0`，防止积分饱和导致的回弹过冲。

### 2. 遥控器软加速斜坡 (`RampAccel`)

```c
float RampAccel(float target, float current, float accel_step) {
    if(target * current < 0)
        return 0;               // 方向切换：先回零，保护机械
    if(fabsf(target) > fabsf(current))
        return current ± step;  // 加速方向：限步长
    return target;              // 减速方向：不限制，保证制动响应
}
```

**精妙之处：**
- **方向切换保护**：`target * current < 0` 时直接返回 0，防止急停→反向导致的机械冲击和轮子打滑。
- **不对称限速**：加速时限步长，减速不限制——保证制动响应速度的同时，起步平滑不甩尾。
- **极简实现**：零状态存储，纯函数，无副作用。

### 3. 三全向轮运动学解算

```c
v1 = +Ex * 0.5 - Ey * 0.866 - L * Eω
v2 = -Ex * 0.5 - Ey * 0.866 + L * Eω
v3 = -Ex * 1.0           - L * Eω
```

**精妙之处：**
- `SQRT3_OVER_2 = 0.866f` 和 `OMNI_WHEEL_FACTOR = 0.5f` 预计算为常量，避免运行时 `sqrt(3)/2` 和 `cos(60°)` 的浮点运算。
- 三全向轮 120° 对称布局的运动学矩阵直接硬编码，比矩阵乘法快得多。

### 4. 气缸击球时序控制

```c
if (hitting == 1) {
    // 0ms:    PA8, PC9 置高
    // 1ms:    PB0, PB1 置高
    // 2ms:    PC4, PC5 置高
    // 300ms:  PA8, PC9 复位
    // 301ms:  PB0, PB1 复位
    // 302ms:  PC4, PC5 复位
    // 800ms:  flag_out = 0 (允许下次触发)
}
```

**精妙之处：**
- **多路气缸分时驱动**：6 个电磁阀分三组依次打开（间隔 1ms），避免同时吸合的瞬时电流峰值拉低电源电压。
- **光电门防抖**：`test` 标志位确保光电门只触发一次，需要 `hitting == 2`（流程走完）才允许再次触发。
- **非阻塞延迟**：用 `vTaskDelay` 而非忙等，不影响其他 RTOS 任务。

### 5. 遥控器前后帧 + 边沿检测

```c
Remote_Control.Second = Remote_Control.First;       // 保存上一帧
Key_Parse(recv_pack.Key, &Remote_Control.First);    // 解析当前帧

#define KEY_RISING_EDGE(cur, last, field) \
    ((cur.field == 1) && (last.field == 0))
```

**精妙之处：**
- 14 个按键用位域结构体 `hw_key_t` 承载，边沿检测宏直接通过字段名访问——代码读起来像英文。
- 前后帧缓存让上升沿/下降沿/长按/双击等高级按键逻辑都只需一行宏。

### 6. PID 速度闭环 + 输出限幅

```c
PID_Control2(wheel_actual, wheel_cmd, &steer.vel_pid);

// 实际速度和实际电流作为反馈量，限幅后通过 CAN 发送
motorCurrentBuf[i] = (int16_t)(steer.vel_pid.pid_out);
MotorSend(&hcan2, 0x200, motorCurrentBuf);
```

**精妙之处：**
- 三个轮子共享同一组 PID 参数（`steer_2.vel_pid = steer_1.vel_pid`），调一次全部生效。
- 输出直接写入 DJI 3508 的电流环（CAN ID 0x200），电机内部做电流闭环。

### 7. CAN 双通道分工

- **CAN1**：IMU 数据（JY61） + 预留扩展
- **CAN2**：三路 M3508 电机控制（ID 0x200），电机数据回传（ID 0x201/0x202/0x203）

双 CAN 隔离 IMU 和电机，避免一个总线故障影响另一路。

### 8. UART 错误自恢复

与 exact_fire 相同的 `HAL_UART_ErrorCallback` 逻辑——按 STM32F4 参考手册要求先读 SR 再读 DR 清除 ORE/NE/FE/PE 错误，然后重启 DMA 接收。遥控信号干扰后自动恢复，不卡死。

### 9. 遥控超时保护

```c
if(xSemaphoreTake(Remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE) {
    // 收到数据：正常更新
} else {
    // 200ms 无数据：清零控制量和按键状态
    Remote_Control.Ex = Ey = Eomega = 0;
    memset(&Remote_Control.First, 0, sizeof(...));
}
```

遥控断连后自动停车，不会维持最后一帧的速度导致机器人失控。

---

## 📁 目录结构

```
volley_hit/
├── Chassis/                # 核心框架层 (从 exact_fire 继承)
│   ├── AutoPilot.c/h       #   五次多项式轨迹规划器（预留）
│   ├── ForceChassis.c/h    #   力控底盘解算器（预留）
│   └── Action_Config.c/h   #   动作配置（预留）
├── lib/                    # 驱动与算法库
│   ├── RMLibHead.h         #   跨平台适配层
│   ├── PID.c/h             #   PID + 模糊PID + PI
│   ├── PID_old.c/h         #   经典位置式PID (PID2)
│   ├── CANDrive.c/h        #   CAN 底层收发
│   ├── motor.c/h           #   DJI 电机数据解析
│   ├── motorEx.c/h         #   DJI M3508 扩展 (串级PID)
│   ├── VESC.c/h            #   VESC CAN 协议
│   ├── RobStride2.c/h      #   RobStride 电机 CAN 协议
│   ├── bsp_dwt.c/h         #   DWT 延时/计时
│   ├── JY61.c/h            #   JY61 姿态传感器
│   ├── hit_ball.c/h        #   气缸击球时序控制
│   ├── Vector.h            #   Vector2D / Vector3D
│   └── Chassis.h           #   底盘通用定义
├── Mytask/
│   ├── Task_Init.c/h       #   业务逻辑：遥控、击球、陀螺仪补偿
│   └── config.h            #   机械/运动参数宏定义
├── Remote_control/         #   自定义遥控通信协议栈
│   ├── comm.c/h            #     协议栈核心
│   ├── comm_stm32_hal_middle.c/h  # HAL 适配层
│   ├── data_poll.c/h       #     数据轮询
│   ├── dataFrame.h         #     数据帧定义
│   ├── hardware.h          #     硬件抽象
│   └── My_list.c/h         #     通用链表
├── Matrix/                 #   矩阵运算库
│   ├── matrix.c/h          #     CMSIS-DSP 包装
│   └── svd.c/h             #     SVD/QR 分解
├── Core/                   #   STM32CubeMX 生成
│   ├── Src/main.c          #     双 CAN + 四 UART 初始化
│   └── Src/freertos.c      #     RTOS 入口
└── Middlewares/             #   FreeRTOS 源码
```

---

## 🔧 硬件配置

| 参数 | 值 | 定义处 |
|------|-----|--------|
| 主控 | STM32F407VGTx, 168MHz | `main.c` |
| RTOS | FreeRTOS 10.x, CMSIS-RTOS2 | `freertos.c` |
| 全向轮半径 | 0.075 m | `config.h` |
| 底盘轴距 | 0.457 m | `config.h` |
| 最大线速度 | 15 m/s | `config.h`: `MAX_VELOCITY` |
| 最大角速度 | 15π rad/s | `config.h`: `MAX_OMEGA` |
| 底盘电机 | DJI M3508 ×3 (CAN2) | `Task_Init.c` |
| 击球执行器 | 气缸 (GPIO 6路) | `Task_Init.c` |
| IMU | JY61 (UART4) | `Task_Init.c` |
| 遥控 | UART5 (DMA) | `Task_Init.c` |
| 陀螺仪死区 | ±阈值 | `config.h`: `GYRO_DEADZONE` |
| 打滑判定阈值 | ±阈值 | `config.h`: `SLIP_THRESHOLD` |
| 校正输出死区 | ±阈值 | `config.h`: `CORR_OUT_DEADZONE` |
| 校正输出最大值 | ±阈值 | `config.h`: `CORR_OUT_MAX` |

---

## 🚀 快速开始

1. 用 **Keil MDK** 或 **STM32CubeIDE** 打开项目
2. 确认 Middlewares/FreeRTOS 路径正确
3. 根据实际硬件标定以下参数：
   - `GYRO_DEADZONE` / `SLIP_THRESHOLD` / `SLIP_DEADZONE`（陀螺仪打滑检测灵敏度）
   - `GYRO_LPF_ALPHA` / `SLIP_LPF_ALPHA` / `CORR_LPF_ALPHA`（三阶滤波强度）
   - `WHEEL1/2/3_GRIP_RATIO`（三个轮子的抓地力比例）
   - `JY61_adjust` 的默认 PID 参数
4. 编译 → 烧录 → 上电
5. 遥控器控制底盘移动，光电门检测到球后自动执行气缸击球

---

## 📝 作者

- **IMU 打滑补偿 / 遥控逻辑 / 气缸控制**：刘家瑞
- **底盘框架 (AutoPilot / ForceChassis)**：刘远钊
- **RMLib 基础库**：Yao (KDRobot)

---

## ⚠️ 注意事项

- JY61 上电后需等待 150ms（`vTaskDelay(pdMS_TO_TICKS(150))`）传感器稳定后再读取
- 抓地力比例 `WHEELx_GRIP_RATIO` 需在地面上实测标定，不同地面差异大
- 气缸的 GPIO 引脚分配与具体硬件接线一致，更换机器人需重新确认 `GPIO_PIN_x` 映射
- 滑移补偿只在 `fabsf(Wz_correction) > 0.01f` 时介入，微小漂移不触发，避免静止时轮子微颤
- 本项目的 AutoPilot/ForceChassis 框架已预留但未在业务逻辑中启用（底盘直接走 PID 速度控制），如需高精度轨迹跟踪可启用
