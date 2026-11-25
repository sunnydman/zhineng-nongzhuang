#pragma once

#include <Arduino.h>

// ============================================================================
// 芒果疏果四轮差速底盘 —— 全局硬件引脚与控制参数总表
// 适配主板：Arduino Mega 2560 + Pro Max 扩展板 (25.11.19 - V3.3)
// 实车尺寸：长 320 mm × 宽 310 mm，四路超声波均相对车身最外围内缩 40 mm
// 场地规格：7 条 400 mm 宽垄沟、6 条 300 mm 宽 × 120 mm 高田垄（短垄 R4 端头 600 mm）
//
// 【现场调参速查】
// - 车在沟内画龙/左右摆动：调小 DEFAULT_KP_CENTER 或调大 DEFAULT_KD_CENTER
// - 车贴墙回正太慢：        调大 DEFAULT_KP_CENTER
// - 90° 拐角外角擦到围栏：  调大 DEFAULT_FRONT_STOP_MM (如 145) 或微调 DEFAULT_TURN_ARC_RATIO (0.35~0.45)
// - 90° 转弯过冲/欠转：     过冲调小 DEFAULT_TURN_KP；毛毯上转不动调大 DEFAULT_TURN_MAX_PWM / MIN_PWM
// - 换垄后进新沟偏早/偏晚： 偏早(靠上一条垄)调大 DEFAULT_CROSS_CENTER_MM；偏晚调小
// - 回启动区停得太浅/太深： 调大/调小 DEFAULT_HOME_DIST_MM (编码器闭环毫米数)
//   以上参数均可在串口用 SET 在线修改，调好后发 SAVE 存入 EEPROM 掉电不丢！
// ============================================================================

// ============================================================================
// 1. 四路直流电机驱动 (两块 L298N，180° 新车头方向实车实测确认)
//    注意：两块 L298N 上 4 个 ENA/ENB 黑色跳帽必须全部拔掉！
// ============================================================================

// 新左前轮 (FL)：接扩展板左上 "直流 E5 24 25 GND" <-> 配对编码器 A12/A13
constexpr uint8_t PIN_MOTOR_FL_EN  = 5;
constexpr uint8_t PIN_MOTOR_FL_IN1 = 24;
constexpr uint8_t PIN_MOTOR_FL_IN2 = 25;

// 新左后轮 (RL)：接扩展板左中 "直流 E6 22 23 GND" <-> 配对编码器 A8/A9
constexpr uint8_t PIN_MOTOR_RL_EN  = 6;
constexpr uint8_t PIN_MOTOR_RL_IN1 = 22;
constexpr uint8_t PIN_MOTOR_RL_IN2 = 23;

// 新右前轮 (FR)：接扩展板右上 "直流 E7 26 27 GND" <-> 配对编码器 A14/A15
constexpr uint8_t PIN_MOTOR_FR_EN  = 7;
constexpr uint8_t PIN_MOTOR_FR_IN1 = 26;
constexpr uint8_t PIN_MOTOR_FR_IN2 = 27;

// 新右后轮 (RR)：接扩展板右中 "直流 E8 28 29 GND" <-> 配对编码器 A10/A11
constexpr uint8_t PIN_MOTOR_RR_EN  = 8;
constexpr uint8_t PIN_MOTOR_RR_IN1 = 28;
constexpr uint8_t PIN_MOTOR_RR_IN2 = 29;

// 电机转向修正：正指令必须使对应车轮朝【新车头前进方向】转动（已逐轮实测确认）
constexpr bool MOTOR_FL_REVERSED = false;
constexpr bool MOTOR_RL_REVERSED = false;
constexpr bool MOTOR_FR_REVERSED = true;
constexpr bool MOTOR_RR_REVERSED = true;

// 单轮 PWM 前馈比例系数（已按四轮同步实测标定；RL 略压低以对齐地面起步静摩擦）
constexpr float MOTOR_FL_SCALE = 0.96f;
constexpr float MOTOR_RL_SCALE = 0.86f;
constexpr float MOTOR_FR_SCALE = 0.85f;
constexpr float MOTOR_RR_SCALE = 1.00f;

// 电机输出保护参数
constexpr int16_t  MOTOR_PWM_MAX           = 240;  // PWM 输出上限（留足毛毯加力余量）
constexpr int16_t  MOTOR_PWM_MIN_EFFECTIVE = 35;   // 非零最小有效占空比，防止死区嗡鸣不转
constexpr uint16_t MOTOR_DIR_DEADTIME_US   = 100;  // 换向前先拉低 EN 的保护死区 (us)，保护 L298N

// ============================================================================
// 2. 四路超声波传感器 (HC-SR04，接扩展板上方 "超声" 4P 座：+5V | Trig | Echo | GND)
// ============================================================================
constexpr uint8_t PIN_US_FRONT_TRIG = A4;  // 前 (F): A4 / A5
constexpr uint8_t PIN_US_FRONT_ECHO = A5;

constexpr uint8_t PIN_US_REAR_TRIG  = A2;  // 后 (B): A2 / A3
constexpr uint8_t PIN_US_REAR_ECHO  = A3;

constexpr uint8_t PIN_US_LEFT_TRIG  = A0;  // 左 (L): A0 / A1
constexpr uint8_t PIN_US_LEFT_ECHO  = A1;

constexpr uint8_t PIN_US_RIGHT_TRIG = A6;  // 右 (R): A6 / A7
constexpr uint8_t PIN_US_RIGHT_ECHO = A7;

// 超声波防串扰分时采样与滤波参数
// 注意：Mega 2560 的 A0-A7(Port F) 无硬件中断，故采用分时单路触发 + 10ms 限超时
// 10ms 超时上限同时保护 Serial1(64B 硬件缓冲) 在 25Hz IMU 数据流下绝不溢出
constexpr uint16_t US_SLOT_INTERVAL_MS = 28;     // 相邻两路触发间隔 (ms)，四路一圈约 112 ms
constexpr uint32_t US_TIMEOUT_US       = 10000;  // 单次测距超时上限 (~1715 mm)
constexpr uint16_t US_MIN_VALID_MM     = 25;     // 小于 25 mm 视为 HC-SR04 盲区/异常
constexpr uint16_t US_MAX_VALID_MM     = 1700;   // 大于 1700 mm 视为开阔无遮挡
constexpr uint8_t  US_STALE_LIMIT      = 3;      // 连续无效 3 次后清空滤波并标记为开阔

// ============================================================================
// 3. 上电自启模式与物理按钮配置
//    AUTO_START_MODE:
//      0 = 上电待机（仅串口或 D2 按钮触发，插 USB 调参时用）
//      1 = 上电倒计时后自动跑【直行 -> 90°转弯 -> 直行】测试（拔掉 USB 放地上即走）
//      2 = 上电倒计时后自动跑【完整 7 垄比赛闭环】
//      3 = 上电脱机循环测试舵机（底盘轮子全程停机，拔掉 USB 放心拿万用表调 6V 供电）
// ============================================================================
constexpr uint8_t  AUTO_START_MODE      = 1;      // 当前设为 1：上电 3 秒后自动跑直线->90°转弯->直行测试 (HWT101CT 20Hz)
constexpr uint16_t AUTO_START_DELAY_MS  = 3000;   // 上电发车倒计时 (ms)
constexpr uint8_t  AUTO_START_LANES     = 7;      // 模式 2（全程比赛）遍历垄沟数（专本组为 7 条沟）
constexpr uint8_t  AUTO_SERVO_TEST_PIN  = 33;     // 模式 3 自动循环测试的舵机引脚 (32 / 33 / 34)

// 模式 1（上电自动 L 形测试）参数：后续可直接在这里改距离和左右转方向
constexpr int16_t  AUTO_LTEST_LEG1_MM   = 800;    // 第 1 段直行距离 (mm)
constexpr bool     AUTO_LTEST_TURN_LEFT = false;  // false=右转 90°，true=左转 90°
constexpr int16_t  AUTO_LTEST_LEG2_MM   = 800;    // 第 2 段直行距离 (mm)
constexpr int8_t   AUTO_LTEST_ARC_SIGN  = 0;      // 0=原地 90°转（空地直观测直线+转角），+1=前向弧线 90°转

// 物理启动/急停按钮（接扩展板 "5V 2" 接口的 D2 与 GND，内部上拉；不接按键无任何影响）
// 空闲时按一下 -> LED 急闪倒计时 1.5s 后跑完全程；运行中再按一下 -> 立即急停！
constexpr bool     ENABLE_START_BUTTON = true;
constexpr uint8_t  PIN_START_BTN       = 2;
constexpr uint16_t BTN_DEBOUNCE_MS     = 40;
constexpr uint16_t BTN_START_DELAY_MS  = 1500;

// ============================================================================
// 4. 垄沟居中循迹参数（按 400mm 通道、320×310mm 车身、超声内缩 40mm 精确推算）
// ============================================================================
constexpr int16_t  DEFAULT_BASE_PWM         = 180;    // 沟内正常巡航速度指令 (VelPID 下约 225 mm/s，提速 50%)
constexpr int16_t  DEFAULT_SLOW_PWM         = 100;    // 接近端头或开阔区降速指令 (约 125 mm/s)
constexpr uint16_t DRIVE_SOFT_START_MS      = 320;    // 起步软加速斜坡时长 (ms)，防止四轮静摩擦差导致起步歪
constexpr int16_t  DRIVE_SOFT_START_PWM     = 55;     // 起步斜坡起始速度指令 (PWM)
constexpr uint16_t DEFAULT_FRONT_SLOW_MM    = 320;    // 前超声 <320mm (物理净空 280mm) 开始平滑减速
constexpr uint16_t DEFAULT_FRONT_STOP_MM    = 140;    // 前超声 <=140mm (物理净空 100mm, 车心距墙 260mm) 停稳起弯
constexpr uint16_t DEFAULT_REAR_STOP_MM     = 130;    // 倒车后超声 <=130mm (物理净空 90mm, 车心距左围栏 250mm) 停稳回库
constexpr uint16_t DEFAULT_SIDE_WALL_MAX_MM = 220;    // 居中 85mm、贴对侧墙约 130mm；>220mm 判定该侧开阔无墙
constexpr uint16_t DEFAULT_SINGLE_WALL_MM   = 85;     // 单侧跟墙目标读数：(400-310)/2 + 40 = 85 mm
constexpr int16_t  DEFAULT_ERR_DEADBAND_MM  = 6;      // 横向居中死区 (±6mm)，抑制超声小抖动
constexpr float    DEFAULT_KP_CENTER        = 0.50f;  // 侧向超声居中比例 (PWM / mm)，画龙则调小
constexpr float    DEFAULT_KD_CENTER        = 0.14f;  // 侧向超声居中微分阻尼，抑制摆动
constexpr int16_t  DEFAULT_MAX_STEER_PWM    = 70;     // 差速纠偏最大输出限幅
constexpr uint8_t  END_STOP_CONFIRM_COUNT   = 2;      // 端头停车需连续 2 帧超声确认，防单帧杂波误停
constexpr uint16_t LANE_END_GUARD_MS        = 400;    // 刚进入新沟前 400ms 屏蔽端头停车，防止入口误触发

// ============================================================================
// 5. 串口陀螺仪 / IMU 配置（接左下角 "陀螺仪" -> Serial1: RX1=D19, TX1=D18）
//    支持两种协议一键切换：
//      IMU_PROTO_WIT_HWT101 (当前启用): 维特单轴高精度 HWT101CT-TTL
//        - 供电：9~36V（接 12V 电池 + 与 Mega 共地），TTL 电平接 R19/T18
//        - 协议：11 字节定长帧 0x55 0x52(Z轴角速度) / 0x55 0x53(Z轴偏航角)，已实测 9600 bps 连通
//      IMU_PROTO_YAHBOOM (备用): 原雅博六轴串口 IMU (0x7E 0x23, 115200 bps)
// ============================================================================
constexpr uint8_t  IMU_PROTO_YAHBOOM        = 0;
constexpr uint8_t  IMU_PROTO_WIT_HWT101     = 1;
constexpr uint8_t  IMU_PROTOCOL             = IMU_PROTO_WIT_HWT101;

constexpr uint32_t IMU_BAUD_RATE            = (IMU_PROTOCOL == IMU_PROTO_WIT_HWT101) ? 9600UL : 115200UL;
constexpr uint8_t  IMU_FRAME_HEAD1          = 0x7E;   // 雅博协议帧头1（备用）
constexpr uint8_t  IMU_FRAME_HEAD2          = 0x23;   // 雅博协议帧头2（备用）
constexpr uint8_t  IMU_FUNC_VERSION         = 0x01;
constexpr uint8_t  IMU_FUNC_RAW_ACCEL       = 0x04;
constexpr uint8_t  IMU_FUNC_QUAT            = 0x16;
constexpr uint8_t  IMU_FUNC_EULER           = 0x26;
constexpr uint8_t  IMU_FUNC_BARO            = 0x32;
constexpr uint8_t  IMU_FUNC_SET_RATE        = 0x60;
constexpr uint8_t  IMU_FUNC_CALIB_IMU       = 0x70;
constexpr uint8_t  IMU_FUNC_REQ_VERSION     = 0x80;
constexpr uint8_t  IMU_FUNC_RETURN_STATE    = 0x81;
constexpr bool     IMU_EULER_IS_RADIANS     = true;
constexpr uint16_t IMU_ONLINE_TIMEOUT_MS    = 500;    // 超过 500ms 无有效偏航帧则判定离线
// 方向约定：车头逆时针(向左)转 90° 时 yaw 应增大到 +90°；烧录后实测若显示 -90° 则改为 true
constexpr bool     IMU_YAW_INVERT           = false;
constexpr float    DEFAULT_KP_YAW_HOLD      = 3.20f;  // 直行航向锁直比例 (PWM / deg)，提高起步及直线抗偏能力

// ============================================================================
// 6. 端头 90° 弧线过弯、跨垄换沟与全程回库参数
//    针对 320×310mm（对角线 445.5mm > 400mm）设计前/后偏置弧线过弯，清障余量 >32mm
// ============================================================================
constexpr int16_t  DEFAULT_TURN_MAX_PWM        = 125;   // 90° 转向最大差速指令（减小旋转惯量，毛毯转不动可加大）
constexpr int16_t  DEFAULT_TURN_MIN_PWM        = 65;    // 接近目标角时的最小起转指令
constexpr float    DEFAULT_TURN_KP             = 1.85f; // 陀螺仪闭环转向比例 (PWM / deg)，过冲则调小
constexpr float    DEFAULT_TURN_KD             = 0.22f; // 陀螺仪 Z 轴角速度阻尼 (PWM / (deg/s))，提前刹车防转过头
constexpr float    DEFAULT_TURN_ARC_RATIO      = 0.35f; // 弧线偏置比 (0=纯原地转；0.35 适配 320×310 避障过弯)
constexpr float    DEFAULT_TURN_TOL_DEG        = 2.0f;  // 转向到位判定容差 (±deg)
constexpr uint16_t DEFAULT_TURN_HOLD_MS        = 120;   // 进入容差后稳停确认时间 (ms)
constexpr uint16_t DEFAULT_TURN_FALLBACK_90_MS = 720;   // IMU 离线时的定时 90° 回退时长 (ms)
constexpr uint16_t DEFAULT_TURN_TIMEOUT_MS     = 3800;  // 单次转向最大超时保护 (ms)

constexpr int16_t  DEFAULT_CROSS_PWM           = 130;   // 沿端头通道跨垄速度指令 (约 162 mm/s)
constexpr uint16_t DEFAULT_CROSS_OUTER_WALL_MM = 85;    // 跨垄时外侧超声贴外围栏目标 (400mm 通道居中 = 85mm)
constexpr uint16_t DEFAULT_CROSS_RIDGE_MAX_MM  = 420;   // 内侧超声识别垄端面阈值 (普通垄~85mm, 短垄R4~285mm)
constexpr uint16_t DEFAULT_CROSS_OPEN_MIN_MM   = 550;   // 内侧超声判定离开垄端面、到达新沟口阈值 (>550mm 或 ---)
constexpr uint8_t  DEFAULT_CROSS_DEBOUNCE_CNT  = 2;     // 边沿跳变连续确认次数，防单帧误触发
constexpr uint16_t DEFAULT_CROSS_MIN_MS        = 250;   // 刚启动跨垄后前 250ms 不判边沿，避开转弯残留区
constexpr uint16_t DEFAULT_CROSS_CENTER_MM     = 140;   // 扫过垄端面后编码器再走 140mm，随后弧线 90° 恰好落入新沟中线
constexpr uint16_t DEFAULT_CROSS_MAX_DIST_MM   = 680;   // 跨垄编码器最大里程兜底 (两沟中心距 700mm，防漏扫冲过头)
constexpr uint16_t DEFAULT_CROSS_CENTER_MS     = 1500;  // 过垄对中微进超时保护 (ms)
constexpr uint16_t DEFAULT_CROSS_MAX_MS        = 5000;  // 跨垄总超时保护 (ms)
constexpr uint16_t DEFAULT_SETTLE_MS           = 200;   // 动作切换间短停等待超声清空刷新的时间 (ms)
constexpr uint16_t DEFAULT_EXIT_MAX_MS         = 25000; // 出启动区沿左通道直冲顶围栏 (4.6m) 最大超时保护 (ms)
constexpr uint16_t DEFAULT_HOME_DIST_MM        = 650;   // 对准入口后编码器直行 650mm，将 320mm 车体完整停入 1000mm 启动区
constexpr uint16_t DEFAULT_HOME_DRIVE_MS       = 6500;  // 回启动区行驶最大超时保护 (ms)

// ============================================================================
// 7. 四路正交编码器与四轮独立速度 PI 闭环 (PCINT2: A8-A15，AB 四倍频)
//    实车确认通道：FL=A12/A13, RL=A8/A9, FR=A14/A15, RR=A10/A11
// ============================================================================
constexpr bool  ENCODER_FL_REVERSED     = true;     // 实测确认：前进时四路计数均为正
constexpr bool  ENCODER_RL_REVERSED     = true;
constexpr bool  ENCODER_FR_REVERSED     = false;
constexpr bool  ENCODER_RR_REVERSED     = false;
constexpr float ENCODER_COUNTS_PER_REV  = 3960.0f;  // 11 PPR × 4 倍频 × 1:90 减速比
constexpr float WHEEL_DIAMETER_MM       = 67.0f;    // 车轮外径 (mm)，若 D 500 实测有误差可按比例微调此值
constexpr uint16_t ENCODER_SAMPLE_MS    = 50;       // 测速与速度闭环周期 (50ms = 20Hz)

// 四轮独立速度 PI 闭环（已实测四轮同步误差 < ±1.9%，自动抗毛毯和电池压降）
constexpr bool     DEFAULT_VEL_PID_ENABLED = true;
constexpr float    DEFAULT_WHEEL_MAX_MMS   = 300.0f;  // 指令满幅 (MOTOR_PWM_MAX) 对应的目标轮速 (mm/s)
constexpr float    DEFAULT_VEL_KP          = 0.32f;   // 速度环比例系数 (加快起步同步响应)
constexpr float    DEFAULT_VEL_KI          = 0.65f;   // 速度环积分系数

// ============================================================================
// 8. 疏果/击打执行机构舵机配置（接扩展板顶部 12 路舵机排针：D32 / D33 / D34）
//    Mega 2560 上 Arduino Servo 库前 12 路独占 Timer5，不干扰 D5(Timer3)/D6-D8(Timer4) 电机 PWM
// ============================================================================
constexpr uint8_t  SERVO_COUNT          = 3;
constexpr uint8_t  PIN_SERVO_0          = 32;    // 舵机 0（D32，待标定具体方位）
constexpr uint8_t  PIN_SERVO_1          = 33;    // 舵机 1（D33，待标定具体方位）
constexpr uint8_t  PIN_SERVO_2          = 34;    // 舵机 2（D34，待标定具体方位）
constexpr uint16_t SERVO_MIN_US         = 500;   // 0° 对应脉宽 (us)，适配常规 180°/270° 舵机
constexpr uint16_t SERVO_MAX_US         = 2500;  // 180° 对应脉宽 (us)
constexpr uint8_t  SERVO_CENTER_DEG     = 90;    // 安全中位角度
constexpr uint8_t  SERVO_TEST_OFFSET    = 25;    // 连通性测试小幅摆动角度 (±25°，防顶死机械限位)
constexpr uint16_t SERVO_TEST_STEP_MS   = 650;   // 每次测试摆动停留时间 (ms)
constexpr float    VEL_INTEGRAL_LIMIT      = 95.0f;   // 积分抗饱和限幅
constexpr uint16_t VEL_FAULT_MS            = 450;     // 编码器断线/反相超过 450ms 自动切回开环防飞车保护

// ============================================================================
// 8. USB 调试串口与遥测
// ============================================================================
constexpr uint32_t SERIAL_BAUD_RATE      = 115200;
constexpr uint16_t TELEMETRY_INTERVAL_MS = 200;
constexpr uint16_t TEST_DEFAULT_DUR_MS   = 1500;
