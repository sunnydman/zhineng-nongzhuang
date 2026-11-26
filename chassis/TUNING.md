# 底盘现场调参行号速查表

所有常用控制参数均集中在 [`include/config.h`](include/config.h)。点击表格中的行号可直接跳转对应代码行。

> **重要提醒（EEPROM 覆盖机制）**
> 若之前在串口执行过 `SAVE`，上电会优先加载 EEPROM 参数覆盖 `config.h` 中的 `DEFAULT_*` 默认值。修改 `config.h` 后要让新默认值生效，二选一：
> 1. 烧录后在串口发送 `RESETCFG`；
> 2. 将 [`src/storage.cpp` L15](src/storage.cpp#L15) 的 `kVersion`（当前 `28`）加 1 后再烧录。

---

## 一、按现场现象快速挑参数

| 现场现象 | 优先去改哪一行 | 参数名 | 调整方向 | 串口在线指令 |
| :--- | :--- | :--- | :--- | :--- |
| 车在沟内画龙 / 左右摆动 | [L191-L194](include/config.h#L191-L194) / [L241-L242](include/config.h#L241-L242) | `CASCADE_KP_DEG_PER_MM` / `CASCADE_KD_DEG_PER_MMS` / `DEFAULT_KP_YAW_HOLD` | 调小外环 KP 或调大外环 KD；IMU 离线时改 [L168-L169](include/config.h#L168-L169) | `SET ykp <值>` / `SET kp <值>` / `SET kd <值>` |
| 车贴墙回正太慢 / 纠偏无力 | [L191-L192](include/config.h#L191-L192) / [L170-L171](include/config.h#L170-L171) | `CASCADE_KP_DEG_PER_MM` / `DEFAULT_MAX_STEER_PWM` | 适当调大；单侧目标偏近/偏远改 [L163-L166](include/config.h#L163-L166) | `SET steer <值>` / `SET side <值>` |
| 第 1 垄(7号沟)末端左侧离上方围栏距离 | [L165-L166](include/config.h#L165-L166) | `DEFAULT_LANE1_LEFT_WALL_MM` | 离上方围栏更远调大，更近调小（仅影响第 1 垄） | 需重烧 |
| 起步太猛 / 加速减速不够柔和 | [L150-L153](include/config.h#L150-L153) | `SPEED_ACCEL_PWM_PER_SEC` / `SPEED_DECEL_PWM_PER_SEC` | 更柔和调小斜率，更干脆调大斜率 | 需重烧 |
| 端头冲过头 / 提前停太远 | [L154-L157](include/config.h#L154-L157) | `DEFAULT_FRONT_SLOW_MM` / `DEFAULT_FRONT_STOP_MM` | 冲过头调大停距或减速距离；停太远调小 | `SET stop <值>` |
| 倒车回库离 1 号垄太近 / 靠左围栏太近 | [L158-L159](include/config.h#L158-L159) | `DEFAULT_REAR_STOP_MM` | 离 1 号垄太近→调小（多退）；离左围栏太近→调大（少退） | `SET rstop <值>` |
| 倒车回库速度偏快 / 想微调 | [L144-L145](include/config.h#L144-L145) | `DEFAULT_REVERSE_PWM` | 走不直时适当调小，想快一点适当调大 | 需重烧 |
| 90° 拐角外角擦到围栏 | [L156-L157](include/config.h#L156-L157) / [L254-L255](include/config.h#L254-L255) | `DEFAULT_FRONT_STOP_MM` / `DEFAULT_TURN_ARC_RATIO` | 调大停距，或开启轻弧线（`0.15 ~ 0.25`） | `SET stop <值>` / `SET arc <值>` |
| 90° 转弯过冲 / 欠转 / 毛毯转不动 | [L247-L253](include/config.h#L247-L253) | `DEFAULT_TURN_MAX_PWM` / `MIN_PWM` / `TURN_KP` / `TURN_KD` | 过冲调小 `KP` 或调大 `KD`；转不动调大 `MAX/MIN_PWM` | `SET tpwm` / `tmin` / `tkp` |
| 出库 / 换垄 / 入库贴外围栏偏近或偏远 | [L265-L270](include/config.h#L265-L270) | `DEFAULT_EXIT/CROSS/HOME_OUTER_WALL_MM` | 贴外围栏太近调大，离外围栏太远（擦田垄）调小 | `SET eout` / `cout` / `hout` |
| 换垄后进新沟偏早 / 偏晚 | [L279-L282](include/config.h#L279-L282) | `DEFAULT_CROSS_CENTER_MM` / `DEFAULT_CROSS_MAX_DIST_MM` | 偏早（靠上一条垄）调大 `CENTER_MM`；偏晚调小 | `SET cmm <值>` / `SET cmax <值>` |
| 回启动区停得太浅 / 太深 | [L289-L290](include/config.h#L289-L290) | `DEFAULT_HOME_DIST_MM` | 太浅调大，太深调小（单位 mm） | `SET hdist <值>` |
| 整车跑太快不稳 / 想提速 | [L142-L147](include/config.h#L142-L147) / [L263-L264](include/config.h#L263-L264) | `DEFAULT_BASE_PWM` / `REVERSE_PWM` / `SLOW_PWM` / `CROSS_PWM` | 同步微调巡航、倒车、降速与跨垄 PWM | `SET spd` / `slow` / `cpwm` |
| 击打杆收回不够竖 / 外扩角度不准 | [L347-L361](include/config.h#L347-L361) | `SERVO_{LD,RD,LU,RU}_{IDLE,STRIKE}_DEG` / `HOLD_MS` | 逐路微调角度与保持时长（需重烧） | 串口 `SV` 单测验证 |

---

## 二、全参数行号总表（[`include/config.h`](include/config.h)）

### 1. 上电模式与测试配置
| 行号 | 参数名 | 当前值 | 串口 `SET` | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| [L112-L114](include/config.h#L112-L114) | `AUTO_START_MODE` | `2` | 需重烧 | `0`待机 / `1`L形测 / `2`完整7垄 / `3`舵机循环 / `4`单垄居中 |
| [L115](include/config.h#L115) | `AUTO_START_DELAY_MS` | `3000` | 需重烧 | 上电发车倒计时（ms） |
| [L116-L117](include/config.h#L116-L117) | `AUTO_START_LANES` | `7` | 需重烧 | 模式 2 遍历垄沟总数 |
| [L118-L119](include/config.h#L118-L119) | `KEEP_HEADING_NO_UTURN` | `true` | 需重烧 | `true`=进沟不掉头（前进/倒车交替）；`false`=传统 U 弯掉头 |
| [L120-L121](include/config.h#L120-L121) | `AUTO_SERVO_TEST_PIN` | `33` | 需重烧 | 模式 3 测试的舵机引脚（32/33/34/35） |
| [L124-L128](include/config.h#L124-L128) | `AUTO_LTEST_LEG1_MM` / `TURN_LEFT` / `LEG2_MM` / `ARC_SIGN` | `800 / false / 800 / 0` | 需重烧 | 模式 1 的 L 形测试两段距离、转向与弧线开关 |
| [L133-L136](include/config.h#L133-L136) | `ENABLE_START_BUTTON` / `PIN` / `DEBOUNCE` / `DELAY` | `true / 2 / 40 / 1500` | 需重烧 | D2 物理启动/急停按钮配置 |

### 2. 直行速度、平顺加减速与端头停距
| 行号 | 参数名 | 当前值 | 串口 `SET` | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| [L144-L145](include/config.h#L144-L145) | `DEFAULT_BASE_PWM` | `170` | `spd` | 沟内巡航指令（VelPID 下约 0.60 m/s） |
| [L146-L147](include/config.h#L146-L147) | `DEFAULT_REVERSE_PWM` | `130` | 需重烧 | 偶数沟倒车过沟与回库专用速度（约 0.46 m/s，稳直过毛毯） |
| [L148-L149](include/config.h#L148-L149) | `DEFAULT_SLOW_PWM` | `75` | `slow` | 端头或开阔区降速指令（约 0.26 m/s） |
| [L151](include/config.h#L151) | `DRIVE_SOFT_START_PWM` | `50` | 需重烧 | 平滑起步初始速度指令 |
| [L152-L153](include/config.h#L152-L153) | `SPEED_ACCEL_PWM_PER_SEC` | `260.0f` | 需重烧 | 统一平滑加速斜率（PWM/s） |
| [L154-L155](include/config.h#L154-L155) | `SPEED_DECEL_PWM_PER_SEC` | `380.0f` | 需重烧 | 统一平滑减速斜率（PWM/s） |
| [L156-L157](include/config.h#L156-L157) | `DEFAULT_FRONT_SLOW_MM` | `580` | 需重烧 | 前超声提前线性减速距离（mm） |
| [L158-L159](include/config.h#L158-L159) | `DEFAULT_FRONT_STOP_MM` | `155` | `stop` | 前超声端头停车阈值（mm） |
| [L160-L161](include/config.h#L160-L161) | `DEFAULT_REAR_STOP_MM` | `135` | `rstop` | 倒车后超声停车阈值（mm，兼顾偶数沟左端头转弯与回库） |

### 3. 垄沟居中、贴墙与串级纠偏
| 行号 | 参数名 | 当前值 | 串口 `SET` | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| [L160-L162](include/config.h#L160-L162) | `DEFAULT_SIDE_WALL_MAX_MM` | `240` | `wall` | 大于此值判定该侧开阔无墙（mm） |
| [L163-L164](include/config.h#L163-L164) | `DEFAULT_SINGLE_WALL_MM` | `96` | `side` | 沟内通用单侧贴墙目标距离（mm） |
| [L165-L166](include/config.h#L165-L166) | `DEFAULT_LANE1_LEFT_WALL_MM` | `106` | 需重烧 | **仅第 1 垄（7 号沟）末端**左侧贴上方围栏目标距离（mm） |
| [L167](include/config.h#L167) | `DEFAULT_ERR_DEADBAND_MM` | `5` | 需重烧 | 横向居中误差死区（±mm） |
| [L168](include/config.h#L168) | `DEFAULT_KP_CENTER` | `0.30f` | `kp` | 侧向超声控距比例（IMU 离线备用） |
| [L169](include/config.h#L169) | `DEFAULT_KD_CENTER` | `0.08f` | `kd` | 侧向超声控距微分阻尼 |
| [L170-L171](include/config.h#L170-L171) | `DEFAULT_MAX_STEER_PWM` | `58` | `steer` | 差速纠偏最大输出限幅 |
| [L174](include/config.h#L174) | `MECANUM_STRAFE_INVERT` | `false` | 需重烧 | 麦轮横移方向反转开关 |
| [L182-L188](include/config.h#L182-L188) | `ALIGN_STRAFE_MAX/MIN_PWM` / `TOL_MM` / `HOLD_MS` / `MAX_MS` | `95 / 65 / 6 / 120 / 1200` | 需重烧 | 发车前原地麦轮对中参数 |
| [L189-L190](include/config.h#L189-L190) | `CASCADE_MAX_YAW_DEG` | `5.0f` | 需重烧 | 串级外环最大切入角限幅（前进 ±5°，倒车自动收紧至 ±2.5°） |
| [L191-L192](include/config.h#L191-L192) | `CASCADE_KP_DEG_PER_MM` | `0.14f` | 需重烧 | 串级外环横向误差→切入角比例 |
| [L193-L194](include/config.h#L193-L194) | `CASCADE_KD_DEG_PER_MMS` | `0.03f` | 需重烧 | 串级外环横向误差微分阻尼 |
| [L195-L199](include/config.h#L195-L199) | `DRIVE_STRAFE_KP` / `DRIVE_STRAFE_MAX_PWM` | `0.00f` / `20` | 需重烧 | 行进间麦轮微横移增益（当前关闭）与限幅 |
| [L200-L202](include/config.h#L200-L202) | `EXIT_CALM_DIST_MM` / `EXIT_CALM_PWM` | `500` / `120` | 需重烧 | 出库大偏差保护距离（mm）与温和限速 |
| [L204-L207](include/config.h#L204-L207) | `END_STOP_CONFIRM_COUNT` / `LANE_END_GUARD_MS` | `2` / `400` | 需重烧 | 端头停车确认帧数 / 进沟入口防误停时长 |
| [L240](include/config.h#L240) | `IMU_YAW_INVERT` | `false` | 需重烧 | 陀螺仪偏航角方向反转开关 |
| [L241-L242](include/config.h#L241-L242) | `DEFAULT_KP_YAW_HOLD` | `2.60f` | `ykp` | 直行串级内环航向锁定比例 |

### 4. 端头 90° 过弯
| 行号 | 参数名 | 当前值 | 串口 `SET` | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| [L247-L248](include/config.h#L247-L248) | `DEFAULT_TURN_MAX_PWM` | `185` | `tpwm` | 90° 转向最大差速指令 |
| [L249-L250](include/config.h#L249-L250) | `DEFAULT_TURN_MIN_PWM` | `95` | `tmin` | 接近目标角时的最小起转指令 |
| [L251](include/config.h#L251) | `DEFAULT_TURN_KP` | `2.50f` | `tkp` | 陀螺仪闭环转向比例（PWM/deg） |
| [L252-L253](include/config.h#L252-L253) | `DEFAULT_TURN_KD` | `0.42f` | 需重烧 | 陀螺仪 Z 轴角速度阻尼（防过冲） |
| [L254-L255](include/config.h#L254-L255) | `DEFAULT_TURN_ARC_RATIO` | `0.00f` | `arc` | 弧线偏置比例（`0.00` = 纯原地转） |
| [L256](include/config.h#L256) | `DEFAULT_TURN_TOL_DEG` | `2.0f` | 需重烧 | 转向到位判定容差（±deg） |
| [L257-L258](include/config.h#L257-L258) | `DEFAULT_TURN_HOLD_MS` | `100` | 需重烧 | 到位后稳停确认时长（ms） |
| [L259-L260](include/config.h#L259-L260) | `DEFAULT_TURN_FALLBACK_90_MS` | `550` | `tms` | IMU 离线时的定时 90° 回退时长（ms） |
| [L261](include/config.h#L261) | `DEFAULT_TURN_TIMEOUT_MS` | `3500` | 需重烧 | 单次转向超时保护（ms） |

### 5. 出库、跨垄换沟与回启动区
| 行号 | 参数名 | 当前值 | 串口 `SET` | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| [L263-L264](include/config.h#L263-L264) | `DEFAULT_CROSS_PWM` | `120` | `cpwm` | 沿端头通道跨垄与入库速度指令（约 0.42 m/s） |
| [L265-L266](include/config.h#L265-L266) | `DEFAULT_EXIT_OUTER_WALL_MM` | `96` | `eout` | **出库**沿左通道直冲顶围栏时贴左外围栏目标（mm） |
| [L267-L268](include/config.h#L267-L268) | `DEFAULT_CROSS_OUTER_WALL_MM` | `90` | `cout` | **换垄**沿端头通道跨垄时贴外围栏目标（mm） |
| [L269-L270](include/config.h#L269-L270) | `DEFAULT_HOME_OUTER_WALL_MM` | `96` | `hout` | **入库**过门时右侧贴左外围栏目标（mm） |
| [L271-L272](include/config.h#L271-L272) | `DEFAULT_CROSS_RIDGE_MAX_MM` | `420` | `cridge` | 内侧超声识别垄端面阈值（mm） |
| [L273-L274](include/config.h#L273-L274) | `DEFAULT_CROSS_OPEN_MIN_MM` | `550` | `copen` | 内侧超声判定离开垄端面阈值（mm） |
| [L275-L278](include/config.h#L275-L278) | `DEFAULT_CROSS_DEBOUNCE_CNT` / `MIN_MS` | `2` / `220` | 需重烧 | 边沿防抖次数 / 起步屏蔽时长（ms） |
| [L279-L280](include/config.h#L279-L280) | `DEFAULT_CROSS_CENTER_MM` | `35` | `cmm` | 离开垄边后的对中微进距离（mm） |
| [L281-L282](include/config.h#L281-L282) | `DEFAULT_CROSS_MAX_DIST_MM` | `625` | `cmax` | 跨垄编码器总里程硬兜底（mm） |
| [L283-L284](include/config.h#L283-L284) | `DEFAULT_CROSS_CENTER_MS` / `MAX_MS` | `1600` / `4500` | `ccent` / 需重烧 | 对中微进超时 / 跨垄总超时保护（ms） |
| [L285-L286](include/config.h#L285-L286) | `DEFAULT_SETTLE_MS` | `90` | 需重烧 | 动作切换短停等超声刷新时间（ms） |
| [L287-L288](include/config.h#L287-L288) | `DEFAULT_EXIT_MAX_MS` | `25000` | 需重烧 | 出库直冲顶围栏最大超时保护（ms） |
| [L289-L290](include/config.h#L289-L290) | `DEFAULT_HOME_DIST_MM` | `650` | `hdist` | 对准入口后直行入库里程（mm） |
| [L291-L292](include/config.h#L291-L292) | `DEFAULT_HOME_DRIVE_MS` | `6500` | `home` | 回启动区最大超时保护（ms） |

### 6. 电机校准、编码器与速度 PI 闭环
| 行号 | 参数名 | 当前值 | 串口 `SET` | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| [L56-L59](include/config.h#L56-L59) | `MOTOR_{FL,RL,FR,RR}_REVERSED` | `F / F / T / T` | 需重烧 | 四轮转向修正 |
| [L63-L66](include/config.h#L63-L66) | `MOTOR_{FL,RL,FR,RR}_SCALE` | `0.84 / 0.92 / 0.91 / 1.00` | `SCALE <轮> <值>` | 单轮 PWM 前馈比例 |
| [L69-L74](include/config.h#L69-L74) | `MOTOR_PWM_MAX` / `MIN_EFFECTIVE` / `DEADTIME_US` | `255 / 35 / 100` | 需重烧 | PWM 上限、最小有效占空比与换向死区 |
| [L299-L302](include/config.h#L299-L302) | `ENCODER_{FL,RL,FR,RR}_REVERSED` | `T / T / F / F` | 需重烧 | 四路编码器极性修正 |
| [L303-L306](include/config.h#L303-L306) | `ENCODER_COUNTS_PER_REV` / `WHEEL_DIAMETER_MM` | `1320.0f` / `75.0f` | 需重烧 | 每圈总脉冲数与麦轮外径标定 |
| [L307](include/config.h#L307) | `ENCODER_SAMPLE_MS` | `50` | 需重烧 | 测速与速度闭环周期（ms） |
| [L311](include/config.h#L311) | `DEFAULT_VEL_PID_ENABLED` | `true` | `vpid` (`0`/`1`) | 四轮独立速度 PI 闭环总开关 |
| [L312-L313](include/config.h#L312-L313) | `DEFAULT_WHEEL_MAX_MMS` | `900.0f` | `vmax` | PWM=255 对应的目标轮速（mm/s） |
| [L314-L316](include/config.h#L314-L316) | `DEFAULT_VEL_KP` / `DEFAULT_VEL_KI` | `0.10f` / `0.25f` | `vkp` / `vki` | 速度环 PI 增益 |
| [L378-L380](include/config.h#L378-L380) | `VEL_INTEGRAL_LIMIT` / `VEL_FAULT_MS` | `95.0f` / `450` | 需重烧 | 积分抗饱和限幅 / 编码器断线保护时长 |

### 7. 四路击打舵机角度与超时
| 行号 | 参数名 | 当前值 | 串口 `SET` | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| [L347-L349](include/config.h#L347-L349) | `SERVO_LD_IDLE_DEG` / `STRIKE_DEG` | `30` / `170` | 需重烧 | 左低（D32）收回角 / 外扩击打角（+140°） |
| [L350-L352](include/config.h#L350-L352) | `SERVO_RD_IDLE_DEG` / `STRIKE_DEG` | `150` / `10` | 需重烧 | 右低（D33）收回角 / 外扩击打角（-140°） |
| [L353-L355](include/config.h#L353-L355) | `SERVO_LU_IDLE_DEG` / `STRIKE_DEG` | `150` / `15` | 需重烧 | 左高（D34）收回角 / 外扩击打角（-135°） |
| [L356-L358](include/config.h#L356-L358) | `SERVO_RU_IDLE_DEG` / `STRIKE_DEG` | `30` / `165` | 需重烧 | 右高（D35）收回角 / 外扩击打角（+135°） |
| [L359-L361](include/config.h#L359-L361) | `SERVO_STRIKE_HOLD_MS` | `480` | 需重烧 | 外扩击打保持时长（ms），到位自动收回 |

### 8. 超声波与串口遥测（一般无需常改）
| 行号 | 参数名 | 当前值 | 说明 |
| :--- | :--- | :--- | :--- |
| [L95-L100](include/config.h#L95-L100) | `US_SLOT_INTERVAL_MS` / `TIMEOUT_US` / `MIN` / `MAX` / `STALE` | `28 / 10000 / 25 / 1700 / 3` | 超声分时采样间隔、超时与有效范围 |
| [L376-L377](include/config.h#L376-L377) | `OPENMV_ONLINE_TIMEOUT_MS` | `1000` | OpenMV 断线判定超时（ms） |
| [L386](include/config.h#L386) | `TELEMETRY_INTERVAL_MS` | `200` | 串口 `U` 实时遥测刷新间隔（ms） |

---

## 三、常用串口在线调参命令（`115200 bps`）

```text
P                  # 查看当前所有生效参数与传感器状态
U                  # 开/关实时遥测（F/B/L/R 超声、Yaw、PWM、轮速）
SET <key> <val>    # 在线修改单个参数（如 SET spd 170、SET eout 96、SET cout 90、SET hout 96）
SCALE <轮> <值>    # 微调单轮前馈比例（如 SCALE fl 0.86）
SAVE               # 将当前参数写入 EEPROM 掉电保存
LOAD               # 从 EEPROM 重新加载
RESETCFG           # 清除 EEPROM，恢复 config.h 默认值
```
