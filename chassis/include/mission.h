#pragma once

#include <Arduino.h>
#include "config.h"
#include "encoder_array.h"
#include "imu_serial.h"
#include "motor_driver.h"
#include "tracker.h"
#include "ultrasonic_array.h"

enum MissionState : uint8_t {
  MS_IDLE = 0,
  MS_DRIVE_DIST,           // 单步编码器定距直行（串口 D <mm>，用于地面标定）
  MS_LTEST_LEG1,           // L形测试 第1段：编码器定距直行 + IMU 0° 锁直
  MS_LTEST_PAUSE1,         // L形测试 停顿稳车（准备转向）
  MS_LTEST_TURN,           // L形测试 转向：IMU 闭环 90° 左/右转
  MS_LTEST_PAUSE2,         // L形测试 停顿稳车（准备第2段直行）
  MS_LTEST_LEG2,           // L形测试 第2段：按转向后新航向编码器定距直行
  MS_EXIT_TO_TOP,          // 出启动区，沿左通道(+Y)贴左围栏直达顶围栏(Ch7 端头)
  MS_SETTLE_BEFORE_ENTER,
  MS_TURN_ENTER_FIRST,     // 前向弧线右转 90° 进入 Ch7 (+X)
  MS_SETTLE_AFTER_ENTER,
  MS_TRACK_LANE,           // 沿当前垄沟居中行驶至端头围栏
  MS_SETTLE_BEFORE_TURN1,
  MS_TURN_1,               // 端头第一次前向弧线 90° 转向
  MS_SETTLE_BEFORE_CROSS,
  MS_CROSS_RIDGE,          // 沿端头通道跨过一条田垄（超声波边沿 + 编码器 140mm 对中）
  MS_SETTLE_BEFORE_TURN2,
  MS_TURN_2,               // 端头第二次前向弧线 90° 转向进入新沟
  MS_SETTLE_AFTER_TURN2,
  MS_RETURN_REVERSE,       // 完成最后一条沟(Ch1)后，沿 Ch1 倒车(-X)靠后超声贴左围栏对准入口
  MS_SETTLE_BEFORE_HOME,
  MS_TURN_HOME,            // 后退偏置弧线右转 90° 面向入口(-Y)
  MS_SETTLE_BEFORE_PARK,
  MS_ENTER_START_ZONE      // 编码器闭环直行 650mm 驶回启动区停稳
};

enum CrossPhase : uint8_t {
  CROSS_SEEK_RIDGE = 0,
  CROSS_PASS_RIDGE,
  CROSS_CENTER_ADVANCE
};

struct MissionParams {
  int16_t turnMaxPwm         = DEFAULT_TURN_MAX_PWM;
  int16_t turnMinPwm         = DEFAULT_TURN_MIN_PWM;
  float turnKp               = DEFAULT_TURN_KP;
  float turnArcRatio         = DEFAULT_TURN_ARC_RATIO;
  float turnTolDeg           = DEFAULT_TURN_TOL_DEG;
  uint16_t turnHoldMs        = DEFAULT_TURN_HOLD_MS;
  uint16_t turnFallback90Ms  = DEFAULT_TURN_FALLBACK_90_MS;
  uint16_t turnTimeoutMs     = DEFAULT_TURN_TIMEOUT_MS;

  int16_t crossPwm           = DEFAULT_CROSS_PWM;
  uint16_t crossOuterWallMm  = DEFAULT_CROSS_OUTER_WALL_MM;
  uint16_t crossRidgeMaxMm   = DEFAULT_CROSS_RIDGE_MAX_MM;
  uint16_t crossOpenMinMm    = DEFAULT_CROSS_OPEN_MIN_MM;
  uint8_t crossDebounceCnt   = DEFAULT_CROSS_DEBOUNCE_CNT;
  uint16_t crossMinMs        = DEFAULT_CROSS_MIN_MS;
  uint16_t crossCenterMm     = DEFAULT_CROSS_CENTER_MM;
  uint16_t crossMaxDistMm    = DEFAULT_CROSS_MAX_DIST_MM;
  uint16_t crossCenterMs     = DEFAULT_CROSS_CENTER_MS;
  uint16_t crossMaxMs        = DEFAULT_CROSS_MAX_MS;
  uint16_t settleMs          = DEFAULT_SETTLE_MS;
  uint16_t exitMaxMs         = DEFAULT_EXIT_MAX_MS;
  uint16_t homeDistMm        = DEFAULT_HOME_DIST_MM;
  uint16_t homeDriveMs       = DEFAULT_HOME_DRIVE_MS;
};

class MissionController {
 public:
  void begin(
      MotorDriver* motors,
      UltrasonicArray* sensors,
      EncoderArray* encoders,
      ImuSerial* imu,
      CorridorTracker* tracker);

  void abort();
  bool isActive() const { return state_ != MS_IDLE; }

  // 动作 0：编码器定距直行（正值前进，负值后退，带 IMU 航向保持与障碍保护）
  void startDriveDistanceMm(int16_t distMm, uint32_t nowMs);

  // 动作 0B：地面综合测试（先直行 leg1Mm -> 左/右转 90° -> 再直行 leg2Mm 停稳）
  // arcSign: 0=原地90°转（空地直观测直线+转角），+1=前向弧线90°转（模拟场地过弯）
  void startLTest(int16_t leg1Mm, bool turnLeft, int16_t leg2Mm, int8_t arcSign, uint32_t nowMs);

  // 动作 1：单独左/右转 90°（arcSign: 0=原地转，+1=前向弧线转，-1=后退弧线转）
  void startTurn90(bool turnLeft, int8_t arcSign, uint32_t nowMs);

  // 动作 2：单独执行一次完整端头 U 形换垄（弧线转 90° -> 超声波边沿+编码器对中跨垄 -> 弧线转 90°）
  void startLaneChange(bool turnLeft, uint32_t nowMs);

  // 动作 3：多垄蛇形自动遍历（例如 7 垄，首端头左/右转，后续左右交替）
  void startSnakeMission(uint8_t totalLanes, bool firstTurnLeft, uint32_t nowMs);

  // 动作 4：完整比赛闭环（启动区出库 -> 沿左通道至 Ch7 -> 蛇形遍历 N 垄 -> Ch1 倒车对口 -> 回启动区）
  void startFullMatch(uint8_t totalLanes, uint32_t nowMs);

  void update(uint32_t nowMs);

  MissionParams& params() { return params_; }
  const MissionParams& params() const { return params_; }
  MissionState state() const { return state_; }
  CrossPhase crossPhase() const { return crossPhase_; }
  uint8_t currentLane() const { return currentLane_; }
  uint8_t totalLanes() const { return totalLanes_; }

  static const char* stateName(MissionState s);
  static const char* crossPhaseName(CrossPhase p);

 private:
  enum JobKind : uint8_t {
    JOB_NONE = 0,
    JOB_DRIVE_DIST,
    JOB_L_TEST,
    JOB_TURN_ONLY,
    JOB_LANE_CHANGE,
    JOB_SNAKE,
    JOB_FULL_MATCH
  };

  void enterState(MissionState next, uint32_t nowMs);
  void startTurnStep(MissionState turnState, float targetYawDeg, bool turnLeft, int8_t arcSign, uint32_t nowMs);
  bool stepDriveDist(uint32_t nowMs);
  bool stepTurn(uint32_t nowMs);
  bool stepExitToTop(uint32_t nowMs);
  bool stepCrossRidge(uint32_t nowMs);
  bool stepEnterStartZone(uint32_t nowMs);

  MotorDriver* motors_ = nullptr;
  UltrasonicArray* sensors_ = nullptr;
  EncoderArray* encoders_ = nullptr;
  ImuSerial* imu_ = nullptr;
  CorridorTracker* tracker_ = nullptr;
  MissionParams params_;

  JobKind job_ = JOB_NONE;
  MissionState state_ = MS_IDLE;
  uint32_t stateEnterMs_ = 0;

  // 多垄状态
  uint8_t currentLane_ = 0;
  uint8_t totalLanes_ = 0;
  bool currentTurnLeft_ = true;
  float nominalHeadingDeg_ = 0.0f;

  // 定距直行与 L 形测试子状态
  float driveStartDistMm_ = 0.0f;
  int16_t driveTargetMm_ = 0;
  int16_t ltestLeg2Mm_ = 0;
  int8_t ltestArcSign_ = 0;

  // 转向子状态
  bool turnLeft_ = true;
  int8_t turnArcSign_ = 0;
  bool turnUseImu_ = false;
  float turnTargetYawDeg_ = 0.0f;
  uint32_t turnInTolSinceMs_ = 0;

  // 跨垄与回库子状态
  CrossPhase crossPhase_ = CROSS_SEEK_RIDGE;
  uint8_t crossEdgeStreak_ = 0;
  uint32_t crossLastInnerMs_ = 0;
  uint32_t crossCenterStartMs_ = 0;
  float crossTotalStartDistMm_ = 0.0f;
  float crossCenterStartDistMm_ = 0.0f;
  float homeStartDistMm_ = 0.0f;
  uint8_t frontStopStreak_ = 0;
  uint32_t lastFrontMs_ = 0;

  // 单侧跟墙 PD 微分状态（用于出库直冲顶围栏与跨垄贴外围栏，平滑抑制画龙）
  bool wallPrevErrValid_ = false;
  int16_t wallPrevErrMm_ = 0;
  uint32_t wallLastUpdateMs_ = 0;
  float wallFilteredDErr_ = 0.0f;
};
