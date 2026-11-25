#include "mission.h"

#include <math.h>

// ============================================================================
// 全程比赛与换垄任务状态机（对应专本组 7 条 400mm 垄沟场地）
//
// 【完整比赛闭环路线（串口发 'F' 或按 D2 启动按钮）】
//   1. MS_EXIT_TO_TOP     : 启动区出库，沿左通道(+Y, 0°)仅贴左外围栏(85mm)直冲至顶围栏(Y=4600)
//   2. MS_TURN_ENTER_FIRST: 前向弧线右转 90°(-90°)，车体稳落第 7 垄(+X)
//   3. MS_TRACK_LANE × 7  : 由第 7 垄自上而下蛇形遍历至第 1 垄（右U弯→Ch6→左U弯→Ch5→...→Ch1）
//      - 每次换垄：MS_TURN_1(弧线90°) -> MS_CROSS_RIDGE(超声边沿+140mm编码器对中, 680mm兜底) -> MS_TURN_2(弧线90°)
//   4. MS_RETURN_REVERSE  : 第 1 垄到达右围栏后，沿第 1 垄倒车居中退回，靠后超声(130mm)停在左围栏前
//   5. MS_TURN_HOME       : 后退偏置弧线右转 90°(180°)，车头面向入口(-Y)并对齐 400mm 入口中线
//   6. MS_ENTER_START_ZONE: 编码器闭环直行 650mm，将 320mm 车体完整停入 1000mm 启动区中央
// ============================================================================

void MissionController::begin(
    MotorDriver* motors,
    UltrasonicArray* sensors,
    EncoderArray* encoders,
    ImuSerial* imu,
    CorridorTracker* tracker) {
  motors_ = motors;
  sensors_ = sensors;
  encoders_ = encoders;
  imu_ = imu;
  tracker_ = tracker;
  abort();
}

void MissionController::abort() {
  job_ = JOB_NONE;
  state_ = MS_IDLE;
  currentLane_ = 0;
  totalLanes_ = 0;
  crossPhase_ = CROSS_SEEK_RIDGE;
  frontStopStreak_ = 0;
  lastFrontMs_ = 0;
  if (tracker_ != nullptr) {
    tracker_->stop();
  } else if (motors_ != nullptr) {
    motors_->stopAll();
  }
}

void MissionController::enterState(MissionState next, uint32_t nowMs) {
  state_ = next;
  stateEnterMs_ = nowMs;
  frontStopStreak_ = 0;
  lastFrontMs_ = 0;
  wallPrevErrValid_ = false;
  wallPrevErrMm_ = 0;
  wallLastUpdateMs_ = 0;
  wallFilteredDErr_ = 0.0f;
}

void MissionController::startDriveDistanceMm(int16_t distMm, uint32_t nowMs) {
  abort();
  if (distMm == 0 || encoders_ == nullptr) {
    return;
  }
  job_ = JOB_DRIVE_DIST;
  driveTargetMm_ = distMm;
  driveStartDistMm_ = encoders_->avgDistanceMm();
  nominalHeadingDeg_ = (imu_ != nullptr && imu_->isOnline(nowMs)) ? imu_->yawDeg() : 0.0f;
  Serial.print(F("[DRIVE] 开始编码器定距直行，目标: "));
  Serial.print(distMm);
  Serial.println(F(" mm"));
  enterState(MS_DRIVE_DIST, nowMs);
}

void MissionController::startLTest(
    int16_t leg1Mm, bool turnLeft, int16_t leg2Mm, int8_t arcSign, uint32_t nowMs) {
  abort();
  if (leg1Mm <= 0 || leg2Mm <= 0 || encoders_ == nullptr) {
    return;
  }
  job_ = JOB_L_TEST;
  currentTurnLeft_ = turnLeft;
  driveTargetMm_ = leg1Mm;
  ltestLeg2Mm_ = leg2Mm;
  ltestArcSign_ = arcSign;
  driveStartDistMm_ = encoders_->avgDistanceMm();

  // 以发车瞬间朝向作为 0.0° 参考基准，便于直接读取第 1 段、转向、第 2 段的航向误差
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    imu_->zeroYaw();
  }
  nominalHeadingDeg_ = 0.0f;

  Serial.print(F("[LTEST] === 启动 直线->转弯->直行 测试: 直行 "));
  Serial.print(leg1Mm);
  Serial.print(F("mm -> "));
  Serial.print(turnLeft ? F("左转90°") : F("右转90°"));
  Serial.print((arcSign != 0) ? F("(弧线)") : F("(原地)"));
  Serial.print(F(" -> 直行 "));
  Serial.print(leg2Mm);
  Serial.println(F("mm ==="));
  enterState(MS_LTEST_LEG1, nowMs);
}

void MissionController::startTurnStep(
    MissionState turnState, float targetYawDeg, bool turnLeft, int8_t arcSign, uint32_t nowMs) {
  turnLeft_ = turnLeft;
  turnArcSign_ = arcSign;
  turnTargetYawDeg_ = ImuSerial::wrapAngleDeg(targetYawDeg);
  turnUseImu_ = (imu_ != nullptr) && imu_->isOnline(nowMs);
  turnInTolSinceMs_ = 0;
  enterState(turnState, nowMs);

  Serial.print(F("[MISSION] "));
  Serial.print(stateName(turnState));
  Serial.print(turnLeft ? F(" 左转90° ") : F(" 右转90° "));
  if (arcSign > 0) Serial.print(F("[前向弧线] "));
  else if (arcSign < 0) Serial.print(F("[后退弧线] "));
  else Serial.print(F("[原地旋转] "));

  if (turnUseImu_) {
    Serial.print(F("目标航向="));
    Serial.print(turnTargetYawDeg_, 1);
    Serial.println(F("°"));
  } else {
    Serial.print(F("IMU离线，降级定时="));
    Serial.print(params_.turnFallback90Ms);
    Serial.println(F(" ms"));
  }
}

void MissionController::startTurn90(bool turnLeft, int8_t arcSign, uint32_t nowMs) {
  abort();
  job_ = JOB_TURN_ONLY;
  currentTurnLeft_ = turnLeft;
  nominalHeadingDeg_ = (imu_ != nullptr && imu_->isOnline(nowMs)) ? imu_->yawDeg() : 0.0f;
  float target = ImuSerial::wrapAngleDeg(nominalHeadingDeg_ + (turnLeft ? 90.0f : -90.0f));
  nominalHeadingDeg_ = target;
  startTurnStep(MS_TURN_1, target, turnLeft, arcSign, nowMs);
}

void MissionController::startLaneChange(bool turnLeft, uint32_t nowMs) {
  abort();
  job_ = JOB_LANE_CHANGE;
  currentTurnLeft_ = turnLeft;
  nominalHeadingDeg_ = (imu_ != nullptr && imu_->isOnline(nowMs)) ? imu_->yawDeg() : 0.0f;
  float target = ImuSerial::wrapAngleDeg(nominalHeadingDeg_ + (turnLeft ? 90.0f : -90.0f));
  nominalHeadingDeg_ = target;
  startTurnStep(MS_TURN_1, target, turnLeft, +1, nowMs);
}

void MissionController::startSnakeMission(uint8_t totalLanes, bool firstTurnLeft, uint32_t nowMs) {
  abort();
  if (totalLanes == 0) {
    return;
  }
  job_ = JOB_SNAKE;
  totalLanes_ = totalLanes;
  currentLane_ = 1;
  currentTurnLeft_ = firstTurnLeft;
  nominalHeadingDeg_ = (imu_ != nullptr && imu_->isOnline(nowMs)) ? imu_->yawDeg() : 0.0f;

  Serial.print(F("[MISSION] 启动多垄蛇形遍历：共 "));
  Serial.print(totalLanes_);
  Serial.print(F(" 垄，首端头="));
  Serial.println(firstTurnLeft ? F("左转") : F("右转"));

  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    tracker_->startForwardWithHeading(nominalHeadingDeg_);
  } else {
    tracker_->startForward(nowMs);
  }
  enterState(MS_TRACK_LANE, nowMs);
}

void MissionController::startFullMatch(uint8_t totalLanes, uint32_t nowMs) {
  abort();
  if (totalLanes == 0) {
    return;
  }
  job_ = JOB_FULL_MATCH;
  totalLanes_ = totalLanes;
  currentLane_ = 0;
  currentTurnLeft_ = false;

  // 发车瞬间把当前朝向锁定为 0° 全局参考零点，后续所有转向均按 90° 整数步进不累积误差
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    imu_->zeroYaw();
  }
  nominalHeadingDeg_ = 0.0f;

  Serial.print(F("[MISSION] === 启动完整比赛闭环 ("));
  Serial.print(totalLanes_);
  Serial.println(F(" 垄) === 出库直冲顶围栏 -> 蛇形回1号沟 -> 倒车对口 -> 回启动区"));
  enterState(MS_EXIT_TO_TOP, nowMs);
}

void MissionController::update(uint32_t nowMs) {
  if (state_ == MS_IDLE || motors_ == nullptr || sensors_ == nullptr || tracker_ == nullptr) {
    return;
  }

  switch (state_) {
    case MS_IDLE:
      return;

    case MS_DRIVE_DIST:
      if (stepDriveDist(nowMs)) {
        motors_->stopAll();
        Serial.print(F("[DRIVE] 结束，编码器实走: "));
        if (encoders_ != nullptr) {
          Serial.print(encoders_->avgDistanceMm() - driveStartDistMm_, 1);
        }
        Serial.println(F(" mm"));
        abort();
      }
      return;

    // ---------------------- L 形测试：直行 -> 90°转弯 -> 直行 ----------------------
    case MS_LTEST_LEG1:
      if (stepDriveDist(nowMs)) {
        bool hitObstacle = (frontStopStreak_ >= END_STOP_CONFIRM_COUNT);
        motors_->stopAll();
        Serial.print(F("[LTEST] 第1段直行完成: 实走="));
        if (encoders_ != nullptr) {
          Serial.print(encoders_->avgDistanceMm() - driveStartDistMm_, 1);
        }
        Serial.print(F(" mm, 当前航向="));
        if (imu_ != nullptr && imu_->isOnline(nowMs)) {
          Serial.print(imu_->yawDeg(), 1);
          Serial.println(F("° (目标 0.0°)"));
        } else {
          Serial.println(F("IMU离线"));
        }
        if (hitObstacle) {
          Serial.println(F("[LTEST] 第1段触发前方超声防撞 -> 安全中止测试"));
          abort();
          return;
        }
        enterState(MS_LTEST_PAUSE1, nowMs);
      }
      return;

    case MS_LTEST_PAUSE1:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs) {
        nominalHeadingDeg_ = ImuSerial::wrapAngleDeg(
            nominalHeadingDeg_ + (currentTurnLeft_ ? 90.0f : -90.0f));
        startTurnStep(MS_LTEST_TURN, nominalHeadingDeg_, currentTurnLeft_, ltestArcSign_, nowMs);
      }
      return;

    case MS_LTEST_TURN:
      if (stepTurn(nowMs)) {
        motors_->stopAll();
        sensors_->clearFilters();
        Serial.print(F("[LTEST] 90°转向完成: 当前航向="));
        if (imu_ != nullptr && imu_->isOnline(nowMs)) {
          Serial.print(imu_->yawDeg(), 1);
          Serial.print(F("° (目标 "));
          Serial.print(nominalHeadingDeg_, 1);
          Serial.println(F("°)"));
        } else {
          Serial.println(F("IMU离线"));
        }
        enterState(MS_LTEST_PAUSE2, nowMs);
      }
      return;

    case MS_LTEST_PAUSE2:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs && sensors_->allSampledOnce()) {
        driveTargetMm_ = ltestLeg2Mm_;
        driveStartDistMm_ = (encoders_ != nullptr) ? encoders_->avgDistanceMm() : 0.0f;
        Serial.print(F("[LTEST] 开始第2段直行，目标: "));
        Serial.print(driveTargetMm_);
        Serial.println(F(" mm"));
        enterState(MS_LTEST_LEG2, nowMs);
      }
      return;

    case MS_LTEST_LEG2:
      if (stepDriveDist(nowMs)) {
        motors_->stopAll();
        Serial.print(F("[LTEST] === 第2段直行完成: 实走="));
        if (encoders_ != nullptr) {
          Serial.print(encoders_->avgDistanceMm() - driveStartDistMm_, 1);
        }
        Serial.print(F(" mm, 最终航向="));
        if (imu_ != nullptr && imu_->isOnline(nowMs)) {
          Serial.print(imu_->yawDeg(), 1);
          Serial.print(F("° (目标 "));
          Serial.print(nominalHeadingDeg_, 1);
          Serial.println(F("°) ==="));
        } else {
          Serial.println(F("IMU离线 ==="));
        }
        abort();
      }
      return;

    // 阶段 1：启动区出发，沿左通道(+Y)直冲至顶围栏(Y=4600)
    case MS_EXIT_TO_TOP:
      if (stepExitToTop(nowMs)) {
        motors_->stopAll();
        enterState(MS_SETTLE_BEFORE_ENTER, nowMs);
      }
      return;

    case MS_SETTLE_BEFORE_ENTER:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs) {
        nominalHeadingDeg_ = ImuSerial::wrapAngleDeg(nominalHeadingDeg_ - 90.0f);
        startTurnStep(MS_TURN_ENTER_FIRST, nominalHeadingDeg_, false, +1, nowMs);
      }
      return;

    // 阶段 2：顶围栏前向弧线右转 90° 进入第 7 垄 (+X)
    case MS_TURN_ENTER_FIRST:
      if (stepTurn(nowMs)) {
        motors_->stopAll();
        sensors_->clearFilters();  // 清空转向前的近墙滤波残留
        enterState(MS_SETTLE_AFTER_ENTER, nowMs);
      }
      return;

    case MS_SETTLE_AFTER_ENTER:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs && sensors_->allSampledOnce()) {
        currentLane_ = 1;
        currentTurnLeft_ = false;  // 第 7 垄(+X)到右围栏后先右U弯进第 6 垄
        Serial.print(F("[MISSION] 进入第 1/"));
        Serial.print(totalLanes_);
        Serial.println(F(" 垄（场地 7 号沟）"));
        if (imu_ != nullptr && imu_->isOnline(nowMs)) {
          tracker_->startForwardWithHeading(nominalHeadingDeg_);
        } else {
          tracker_->startForward(nowMs);
        }
        enterState(MS_TRACK_LANE, nowMs);
      }
      return;

    // 阶段 3：垄沟内双环居中直行，到达端头围栏后转入 U 形换垄或回库
    case MS_TRACK_LANE:
      if (tracker_->hasReachedEnd()) {
        if (currentLane_ >= totalLanes_) {
          if (job_ == JOB_FULL_MATCH) {
            if ((totalLanes_ % 2) == 1) {
              // 奇数条沟(如 7 沟)结束于 1 号沟右围栏 -> 倒车沿 1 号沟退回左围栏入口处
              Serial.println(F("[MISSION] 最后一条沟完成 -> 沿 1 号沟倒车对准左围栏入口"));
              if (imu_ != nullptr && imu_->isOnline(nowMs)) {
                tracker_->startReverseWithHeading(nominalHeadingDeg_);
              } else {
                tracker_->startReverse(nowMs);
              }
              enterState(MS_RETURN_REVERSE, nowMs);
            } else {
              // 偶数条沟结束于左围栏 -> 直接左转 90° 面向启动区
              Serial.println(F("[MISSION] 最后一条沟完成(左围栏) -> 准备左转入库"));
              enterState(MS_SETTLE_BEFORE_HOME, nowMs);
            }
            return;
          }
          Serial.print(F("[MISSION] 已跑完全部 "));
          Serial.print(totalLanes_);
          Serial.println(F(" 垄 -> 任务完成停机"));
          abort();
          return;
        }
        Serial.print(F("[MISSION] 第 "));
        Serial.print(currentLane_);
        Serial.println(F(" 垄到端头 -> 准备弧线换垄"));
        motors_->stopAll();
        enterState(MS_SETTLE_BEFORE_TURN1, nowMs);
      }
      return;

    case MS_SETTLE_BEFORE_TURN1:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs) {
        nominalHeadingDeg_ = ImuSerial::wrapAngleDeg(
            nominalHeadingDeg_ + (currentTurnLeft_ ? 90.0f : -90.0f));
        startTurnStep(MS_TURN_1, nominalHeadingDeg_, currentTurnLeft_, +1, nowMs);
      }
      return;

    case MS_TURN_1:
      if (stepTurn(nowMs)) {
        motors_->stopAll();
        if (job_ == JOB_TURN_ONLY) {
          Serial.println(F("[MISSION] 单次 90° 转向完成"));
          abort();
          return;
        }
        sensors_->clearFilters();
        enterState(MS_SETTLE_BEFORE_CROSS, nowMs);
      }
      return;

    case MS_SETTLE_BEFORE_CROSS:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs && sensors_->allSampledOnce()) {
        crossPhase_ = CROSS_SEEK_RIDGE;
        crossEdgeStreak_ = 0;
        crossLastInnerMs_ = 0;
        crossCenterStartMs_ = 0;
        crossTotalStartDistMm_ = (encoders_ != nullptr) ? encoders_->avgDistanceMm() : 0.0f;
        crossCenterStartDistMm_ = crossTotalStartDistMm_;
        Serial.println(F("[MISSION] 开始跨垄（内侧超声扫垄端面 + 编码器 140mm 对中 / 680mm 兜底）"));
        enterState(MS_CROSS_RIDGE, nowMs);
      }
      return;

    case MS_CROSS_RIDGE:
      if (stepCrossRidge(nowMs)) {
        motors_->stopAll();
        enterState(MS_SETTLE_BEFORE_TURN2, nowMs);
      }
      return;

    case MS_SETTLE_BEFORE_TURN2:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs) {
        nominalHeadingDeg_ = ImuSerial::wrapAngleDeg(
            nominalHeadingDeg_ + (currentTurnLeft_ ? 90.0f : -90.0f));
        startTurnStep(MS_TURN_2, nominalHeadingDeg_, currentTurnLeft_, +1, nowMs);
      }
      return;

    case MS_TURN_2:
      if (stepTurn(nowMs)) {
        motors_->stopAll();
        sensors_->clearFilters();
        enterState(MS_SETTLE_AFTER_TURN2, nowMs);
      }
      return;

    case MS_SETTLE_AFTER_TURN2:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs && sensors_->allSampledOnce()) {
        if (job_ == JOB_LANE_CHANGE) {
          Serial.println(F("[MISSION] 单次 U 形换垄完成"));
          abort();
          return;
        }
        currentLane_++;
        currentTurnLeft_ = !currentTurnLeft_;
        Serial.print(F("[MISSION] 进入第 "));
        Serial.print(currentLane_);
        Serial.print('/');
        Serial.print(totalLanes_);
        Serial.println(F(" 垄"));
        if (imu_ != nullptr && imu_->isOnline(nowMs)) {
          tracker_->startForwardWithHeading(nominalHeadingDeg_);
        } else {
          tracker_->startForward(nowMs);
        }
        enterState(MS_TRACK_LANE, nowMs);
      }
      return;

    // 阶段 4：沿 1 号沟倒车至左围栏前（后超声 130mm 停稳）
    case MS_RETURN_REVERSE:
      if (tracker_->hasReachedEnd()) {
        motors_->stopAll();
        Serial.println(F("[MISSION] 倒车已对齐入口上方 -> 准备转向回库"));
        enterState(MS_SETTLE_BEFORE_HOME, nowMs);
      }
      return;

    case MS_SETTLE_BEFORE_HOME:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs) {
        // 奇数沟倒车停在 X≈250 朝向 +X：后退偏置弧线右转 90°(-1) 刚好收正进 400mm 入口中线
        // 偶数沟正向停在 X≈260 朝向 -X：前向偏置弧线左转 90°(+1) 收正进入口中线
        bool turnLeftToHome = ((totalLanes_ % 2) == 0);
        int8_t arcSign = turnLeftToHome ? +1 : -1;
        nominalHeadingDeg_ = 180.0f;
        startTurnStep(MS_TURN_HOME, nominalHeadingDeg_, turnLeftToHome, arcSign, nowMs);
      }
      return;

    // 阶段 5：转向面向启动区 (-Y, 180°)
    case MS_TURN_HOME:
      if (stepTurn(nowMs)) {
        motors_->stopAll();
        sensors_->clearFilters();
        enterState(MS_SETTLE_BEFORE_PARK, nowMs);
      }
      return;

    case MS_SETTLE_BEFORE_PARK:
      motors_->stopAll();
      if ((nowMs - stateEnterMs_) >= params_.settleMs && sensors_->allSampledOnce()) {
        homeStartDistMm_ = (encoders_ != nullptr) ? encoders_->avgDistanceMm() : 0.0f;
        Serial.print(F("[MISSION] 驶入启动区，编码器目标里程: "));
        Serial.print(params_.homeDistMm);
        Serial.println(F(" mm"));
        enterState(MS_ENTER_START_ZONE, nowMs);
      }
      return;

    // 阶段 6：编码器闭环直行 650mm 完整停入启动区
    case MS_ENTER_START_ZONE:
      if (stepEnterStartZone(nowMs)) {
        motors_->stopAll();
        Serial.println(F("[MISSION] === 全程比赛闭环完成，已自主停稳在启动区！==="));
        abort();
      }
      return;
  }
}

bool MissionController::stepDriveDist(uint32_t nowMs) {
  const bool forward = (driveTargetMm_ > 0);
  const float targetAbs = fabs(static_cast<float>(driveTargetMm_));
  const float traveled = (encoders_ != nullptr) ? (encoders_->avgDistanceMm() - driveStartDistMm_) : 0.0f;
  const float progress = forward ? traveled : -traveled;

  if (progress >= targetAbs) {
    return true;
  }
  if ((nowMs - stateEnterMs_) >= 15000UL) {
    Serial.println(F("[WARN] 定距直行超时停机"));
    return true;
  }

  // 双帧防抖超声防撞保护
  const auto& lead = sensors_->get(forward ? US_FRONT : US_REAR);
  const uint16_t stopMm = forward ? tracker_->params().frontStopMm : tracker_->params().rearStopMm;
  if (lead.lastUpdateMs != lastFrontMs_) {
    lastFrontMs_ = lead.lastUpdateMs;
    if (lead.valid && lead.filteredMm <= stopMm) {
      frontStopStreak_++;
    } else {
      frontStopStreak_ = 0;
    }
  }
  if (frontStopStreak_ >= END_STOP_CONFIRM_COUNT) {
    Serial.println(F("[DRIVE] 触发超声防撞保护提前停机"));
    return true;
  }

  const auto& tp = tracker_->params();
  float rem = targetAbs - progress;
  int16_t spd = (rem < 180.0f) ? tp.slowPwm : tp.basePwm;

  // 起步软加速斜坡（DRIVE_SOFT_START_MS）：让四轮从低速同步进入速度闭环，抑制静摩擦差异引起的起步歪斜
  const uint32_t elapsedMs = nowMs - stateEnterMs_;
  if (elapsedMs < DRIVE_SOFT_START_MS && spd > DRIVE_SOFT_START_PWM) {
    int32_t ramp = static_cast<int32_t>(DRIVE_SOFT_START_PWM) +
                   (static_cast<int32_t>(spd - DRIVE_SOFT_START_PWM) * static_cast<int32_t>(elapsedMs)) /
                       static_cast<int32_t>(DRIVE_SOFT_START_MS);
    spd = static_cast<int16_t>(ramp);
  }

  int16_t cmdFwd = forward ? spd : -spd;

  int16_t yawTurn = 0;
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    float yawErr = ImuSerial::wrapAngleDeg(nominalHeadingDeg_ - imu_->yawDeg());
    yawTurn = -static_cast<int16_t>(tp.kpYawHold * yawErr);
    yawTurn = constrain(yawTurn, -tp.maxSteerPwm, tp.maxSteerPwm);
  }

  motors_->setTank(cmdFwd, yawTurn);
  return false;
}

bool MissionController::stepExitToTop(uint32_t nowMs) {
  const auto& front = sensors_->get(US_FRONT);
  const auto& left = sensors_->get(US_LEFT);
  const auto& tp = tracker_->params();

  // 顶围栏双帧防抖确认，避免 4.6m 直冲途中被单帧噪点误停
  if ((nowMs - stateEnterMs_) >= LANE_END_GUARD_MS && front.lastUpdateMs != lastFrontMs_) {
    lastFrontMs_ = front.lastUpdateMs;
    if (front.valid && front.filteredMm <= tp.frontStopMm) {
      frontStopStreak_++;
    } else {
      frontStopStreak_ = 0;
    }
  }
  if (frontStopStreak_ >= END_STOP_CONFIRM_COUNT) {
    Serial.print(F("[EXIT] 到达顶围栏，前超声="));
    Serial.print(front.filteredMm);
    Serial.println(F(" mm"));
    return true;
  }
  if ((nowMs - stateEnterMs_) >= params_.exitMaxMs) {
    Serial.println(F("[WARN] 直冲顶围栏超时 -> 切入下一阶段"));
    return true;
  }

  int16_t speed = tp.basePwm;
  if (front.valid && front.filteredMm < tp.frontSlowMm && tp.frontSlowMm > tp.frontStopMm) {
    int32_t span = static_cast<int32_t>(tp.frontSlowMm - tp.frontStopMm);
    int32_t rem = (front.filteredMm > tp.frontStopMm)
                      ? static_cast<int32_t>(front.filteredMm - tp.frontStopMm)
                      : 0;
    speed = static_cast<int16_t>(
        tp.slowPwm + (static_cast<int32_t>(tp.basePwm - tp.slowPwm) * rem) / span);
  }
  const uint32_t elapsedMs = nowMs - stateEnterMs_;
  if (elapsedMs < DRIVE_SOFT_START_MS && speed > DRIVE_SOFT_START_PWM) {
    speed = static_cast<int16_t>(
        DRIVE_SOFT_START_PWM +
        (static_cast<int32_t>(speed - DRIVE_SOFT_START_PWM) * static_cast<int32_t>(elapsedMs)) /
            static_cast<int32_t>(DRIVE_SOFT_START_MS));
  }

  // 只跟左侧连续外围栏 (实测居中 70mm) PD 控距 + HWT101CT 0° 航向锁直，无视右侧 6 条田垄开口变化
  int16_t usTurn = 0;
  if (left.valid && left.filteredMm <= tp.sideWallMaxMm) {
    int16_t err = static_cast<int16_t>(left.filteredMm) - static_cast<int16_t>(params_.crossOuterWallMm);
    if (abs(err) <= tp.errDeadbandMm) {
      err = 0;
    }
    if (left.lastUpdateMs != wallLastUpdateMs_) {
      if (wallPrevErrValid_ && wallLastUpdateMs_ != 0 && left.lastUpdateMs > wallLastUpdateMs_) {
        float dtSec = static_cast<float>(left.lastUpdateMs - wallLastUpdateMs_) * 0.001f;
        if (dtSec >= 0.02f && dtSec <= 0.50f) {
          float rawD = static_cast<float>(err - wallPrevErrMm_) / dtSec;
          wallFilteredDErr_ = 0.5f * rawD + 0.5f * wallFilteredDErr_;
        }
      } else {
        wallFilteredDErr_ = 0.0f;
      }
      wallPrevErrMm_ = err;
      wallPrevErrValid_ = true;
      wallLastUpdateMs_ = left.lastUpdateMs;
    }
    usTurn = -static_cast<int16_t>(tp.kp * static_cast<float>(err) + tp.kd * wallFilteredDErr_);
    usTurn = constrain(usTurn, -45, 45);
  } else {
    wallPrevErrValid_ = false;
    wallFilteredDErr_ = 0.0f;
  }

  int16_t yawTurn = 0;
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    float yawErr = ImuSerial::wrapAngleDeg(nominalHeadingDeg_ - imu_->yawDeg());
    yawTurn = -static_cast<int16_t>(tp.kpYawHold * yawErr);
  }

  int16_t cmdTurn = constrain(usTurn + yawTurn, -tp.maxSteerPwm, tp.maxSteerPwm);
  motors_->setTank(speed, cmdTurn);
  return false;
}

bool MissionController::stepEnterStartZone(uint32_t nowMs) {
  const auto& front = sensors_->get(US_FRONT);
  const auto& right = sensors_->get(US_RIGHT);
  const auto& tp = tracker_->params();

  if (front.lastUpdateMs != lastFrontMs_) {
    lastFrontMs_ = front.lastUpdateMs;
    if (front.valid && front.filteredMm <= tp.frontStopMm) {
      frontStopStreak_++;
    } else {
      frontStopStreak_ = 0;
    }
  }
  if (frontStopStreak_ >= END_STOP_CONFIRM_COUNT) {
    Serial.println(F("[HOME] 前方遇阻 -> 提前停机"));
    return true;
  }

  // 优先按编码器精确行驶 homeDistMm (650mm)，超时作为安全兜底
  if (encoders_ != nullptr) {
    float movedMm = encoders_->avgDistanceMm() - homeStartDistMm_;
    if (movedMm >= static_cast<float>(params_.homeDistMm)) {
      Serial.print(F("[HOME] 编码器走满入库距离: "));
      Serial.print(movedMm, 1);
      Serial.println(F(" mm"));
      return true;
    }
  }
  if ((nowMs - stateEnterMs_) >= params_.homeDriveMs) {
    Serial.println(F("[HOME] 达到最大入库时长 -> 停机"));
    return true;
  }

  // 面向 -Y 驶出入口时，右侧仍能短暂看到左外围栏 (X=0)，贴 85mm 居中过门
  int16_t usTurn = 0;
  if (right.valid && right.filteredMm <= tp.sideWallMaxMm) {
    int16_t err = static_cast<int16_t>(right.filteredMm) - static_cast<int16_t>(params_.crossOuterWallMm);
    if (abs(err) > tp.errDeadbandMm) {
      usTurn = static_cast<int16_t>(tp.kp * static_cast<float>(err));
    }
  }

  int16_t yawTurn = 0;
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    float yawErr = ImuSerial::wrapAngleDeg(nominalHeadingDeg_ - imu_->yawDeg());
    yawTurn = -static_cast<int16_t>(tp.kpYawHold * yawErr);
  }

  int16_t cmdTurn = constrain(usTurn + yawTurn, -tp.maxSteerPwm, tp.maxSteerPwm);
  motors_->setTank(params_.crossPwm, cmdTurn);
  return false;
}

bool MissionController::stepTurn(uint32_t nowMs) {
  if ((nowMs - stateEnterMs_) >= params_.turnTimeoutMs) {
    Serial.println(F("[WARN] 转向超时 -> 强制结束本步"));
    motors_->stopAll();
    return true;
  }

  // 模式 A：IMU 角度 PD 闭环转向 + 到位主动电磁刹车（防惯性过冲）
  if (turnUseImu_ && imu_ != nullptr && imu_->isOnline(nowMs)) {
    float errDeg = ImuSerial::wrapAngleDeg(turnTargetYawDeg_ - imu_->yawDeg());
    if (fabs(errDeg) <= params_.turnTolDeg) {
      // 到位时使用 L298N 短接制动（brakeAll）立刻刹住旋转惯性，稳停确认后再释放为 stopAll
      motors_->brakeAll();
      if (turnInTolSinceMs_ == 0) {
        turnInTolSinceMs_ = nowMs;
      } else if ((nowMs - turnInTolSinceMs_) >= params_.turnHoldMs) {
        motors_->stopAll();
        return true;
      }
      return false;
    }

    turnInTolSinceMs_ = 0;
    // PD 转向控制：errDeg > 0 需左转(turnPwm < 0)，gyroZ > 0 为正在左转 -> +KD*gyroZ 提供提前阻尼减速
    float rawTurn = -errDeg * params_.turnKp + DEFAULT_TURN_KD * imu_->gyroZDegPerSec();
    int16_t turnPwm = static_cast<int16_t>(rawTurn);

    // 接近目标角最后 12° 内：若仍在快速旋转（|gyroZ| > 22 deg/s）且阻尼已拉低输出，则主动短刹消惯性；
    // 只有当车身基本慢下来（|gyroZ| <= 22 deg/s）仍未进容差时，才给最小起转 PWM 轻推对正，彻底防转过头
    const float gz = imu_->gyroZDegPerSec();
    if (fabs(errDeg) > 12.0f) {
      int16_t mag = constrain(abs(turnPwm), params_.turnMinPwm, params_.turnMaxPwm);
      turnPwm = (errDeg > 0.0f) ? -mag : mag;
    } else {
      bool movingTowardTarget = (errDeg > 0.0f && gz > 22.0f) || (errDeg < 0.0f && gz < -22.0f);
      if (movingTowardTarget && abs(turnPwm) < params_.turnMinPwm) {
        motors_->brakeAll();
        return false;
      }
      if (abs(turnPwm) < params_.turnMinPwm) {
        turnPwm = (errDeg > 0.0f) ? -params_.turnMinPwm : params_.turnMinPwm;
      } else {
        turnPwm = constrain(turnPwm, -params_.turnMaxPwm, params_.turnMaxPwm);
      }
    }

    // 接近目标角最后 12° 内撤销前/后弧线偏置，改为纯原地精准对正
    int16_t arcFwd = 0;
    if (turnArcSign_ != 0 && fabs(errDeg) > 12.0f) {
      arcFwd = static_cast<int16_t>(
          static_cast<float>(turnArcSign_) * params_.turnArcRatio * static_cast<float>(abs(turnPwm)));
    }

    motors_->setTank(arcFwd, turnPwm);
    return false;
  }

  // 模式 B：IMU 离线时的定时回退方案
  if ((nowMs - stateEnterMs_) >= params_.turnFallback90Ms) {
    motors_->stopAll();
    return true;
  }
  int16_t turnPwm = turnLeft_ ? -params_.turnMaxPwm : params_.turnMaxPwm;
  int16_t arcFwd = static_cast<int16_t>(
      static_cast<float>(turnArcSign_) * params_.turnArcRatio * static_cast<float>(params_.turnMaxPwm));
  motors_->setTank(arcFwd, turnPwm);
  return false;
}

bool MissionController::stepCrossRidge(uint32_t nowMs) {
  const uint32_t elapsed = nowMs - stateEnterMs_;

  // 安全保护 1：前方到上下边界围栏立刻双帧确认停住，绝不撞墙
  const auto& front = sensors_->get(US_FRONT);
  if (front.lastUpdateMs != lastFrontMs_) {
    lastFrontMs_ = front.lastUpdateMs;
    if (front.valid && front.filteredMm <= tracker_->params().frontStopMm) {
      frontStopStreak_++;
    } else {
      frontStopStreak_ = 0;
    }
  }
  if (frontStopStreak_ >= END_STOP_CONFIRM_COUNT) {
    Serial.print(F("[CROSS] 前方到边界围栏 ("));
    Serial.print(front.filteredMm);
    Serial.println(F(" mm) -> 结束跨垄"));
    return true;
  }

  // 安全保护 2：编码器跨垄总里程兜底（两沟中心距 700mm，防短垄漏扫冲过头）
  if (encoders_ != nullptr) {
    float totalCrossMm = encoders_->avgDistanceMm() - crossTotalStartDistMm_;
    if (totalCrossMm >= static_cast<float>(params_.crossMaxDistMm)) {
      Serial.print(F("[CROSS] 达到编码器最大跨垄兜底里程 ("));
      Serial.print(totalCrossMm, 1);
      Serial.println(F(" mm) -> 准备转入新沟"));
      return true;
    }
  }

  // 安全保护 3：最大跨垄时间上限
  if (elapsed >= params_.crossMaxMs) {
    Serial.println(F("[WARN] 跨垄超时 -> 结束跨垄"));
    return true;
  }

  // 确定外侧（贴外围栏 85mm）和内侧（扫田垄端面）超声探头
  const UltrasonicId outerId = currentTurnLeft_ ? US_RIGHT : US_LEFT;
  const UltrasonicId innerId = currentTurnLeft_ ? US_LEFT : US_RIGHT;
  const auto& outer = sensors_->get(outerId);
  const auto& inner = sensors_->get(innerId);

  // 1. 保持沿端头通道居中直行：外侧超声贴 85mm + IMU 航向锁直
  int16_t usTurn = 0;
  if (outer.valid && outer.filteredMm <= tracker_->params().sideWallMaxMm) {
    int16_t outerErr = static_cast<int16_t>(outer.filteredMm) - static_cast<int16_t>(params_.crossOuterWallMm);
    if (abs(outerErr) > tracker_->params().errDeadbandMm) {
      int16_t corr = static_cast<int16_t>(tracker_->params().kp * static_cast<float>(outerErr));
      usTurn = currentTurnLeft_ ? corr : -corr;
    }
  }

  int16_t yawTurn = 0;
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    float yawErr = ImuSerial::wrapAngleDeg(nominalHeadingDeg_ - imu_->yawDeg());
    yawTurn = -static_cast<int16_t>(tracker_->params().kpYawHold * yawErr);
  }

  int16_t cmdTurn = constrain(usTurn + yawTurn, -tracker_->params().maxSteerPwm, tracker_->params().maxSteerPwm);
  motors_->setTank(params_.crossPwm, cmdTurn);

  // 2. 内侧超声边沿状态机：SEEK_RIDGE(找垄) -> PASS_RIDGE(过垄) -> CENTER_ADVANCE(编码器走140mm对中)
  if (crossPhase_ == CROSS_CENTER_ADVANCE) {
    if (encoders_ != nullptr) {
      float movedMm = encoders_->avgDistanceMm() - crossCenterStartDistMm_;
      if (movedMm >= static_cast<float>(params_.crossCenterMm)) {
        Serial.print(F("[CROSS] 过垄后编码器对中完成 (+"));
        Serial.print(movedMm, 1);
        Serial.println(F(" mm) -> 准备弧线转入新沟"));
        return true;
      }
    }
    if ((nowMs - crossCenterStartMs_) >= params_.crossCenterMs) {
      Serial.println(F("[CROSS] 对中微进超时 -> 结束跨垄"));
      return true;
    }
    return false;
  }

  if (elapsed < params_.crossMinMs) {
    return false;
  }

  if (inner.lastUpdateMs == crossLastInnerMs_) {
    return false;
  }
  crossLastInnerMs_ = inner.lastUpdateMs;

  const bool seesRidge = inner.valid && (inner.filteredMm <= params_.crossRidgeMaxMm);
  const bool seesOpen = (!inner.valid) || (inner.filteredMm >= params_.crossOpenMinMm);

  if (crossPhase_ == CROSS_SEEK_RIDGE) {
    if (seesRidge) {
      crossEdgeStreak_++;
      if (crossEdgeStreak_ >= params_.crossDebounceCnt) {
        crossPhase_ = CROSS_PASS_RIDGE;
        crossEdgeStreak_ = 0;
        Serial.print(F("[CROSS] 扫到垄端面 ("));
        Serial.print(inner.filteredMm);
        Serial.println(F(" mm) -> 进入过垄阶段"));
      }
    } else {
      crossEdgeStreak_ = 0;
    }
  } else if (crossPhase_ == CROSS_PASS_RIDGE) {
    if (seesOpen) {
      crossEdgeStreak_++;
      if (crossEdgeStreak_ >= params_.crossDebounceCnt) {
        crossPhase_ = CROSS_CENTER_ADVANCE;
        crossEdgeStreak_ = 0;
        crossCenterStartMs_ = nowMs;
        crossCenterStartDistMm_ = (encoders_ != nullptr) ? encoders_->avgDistanceMm() : 0.0f;
        Serial.println(F("[CROSS] 离开垄端面进入新沟口 -> 编码器微进 140mm 对中"));
      }
    } else {
      crossEdgeStreak_ = 0;
    }
  }

  return false;
}

const char* MissionController::stateName(MissionState s) {
  switch (s) {
    case MS_IDLE:                 return "IDLE";
    case MS_DRIVE_DIST:           return "DRIVE";
    case MS_LTEST_LEG1:           return "LT_LEG1";
    case MS_LTEST_PAUSE1:         return "LT_P1";
    case MS_LTEST_TURN:           return "LT_TURN";
    case MS_LTEST_PAUSE2:         return "LT_P2";
    case MS_LTEST_LEG2:           return "LT_LEG2";
    case MS_EXIT_TO_TOP:          return "EXIT";
    case MS_SETTLE_BEFORE_ENTER:  return "PRE_EN";
    case MS_TURN_ENTER_FIRST:     return "TURN_EN";
    case MS_SETTLE_AFTER_ENTER:   return "POST_EN";
    case MS_TRACK_LANE:           return "LANE";
    case MS_SETTLE_BEFORE_TURN1:  return "PRE_T1";
    case MS_TURN_1:               return "TURN1";
    case MS_SETTLE_BEFORE_CROSS:  return "PRE_CR";
    case MS_CROSS_RIDGE:          return "CROSS";
    case MS_SETTLE_BEFORE_TURN2:  return "PRE_T2";
    case MS_TURN_2:               return "TURN2";
    case MS_SETTLE_AFTER_TURN2:   return "POST_T2";
    case MS_RETURN_REVERSE:       return "RET_REV";
    case MS_SETTLE_BEFORE_HOME:   return "PRE_HM";
    case MS_TURN_HOME:            return "TURN_HM";
    case MS_SETTLE_BEFORE_PARK:   return "PRE_PK";
    case MS_ENTER_START_ZONE:     return "PARK";
    default:                      return "?";
  }
}

const char* MissionController::crossPhaseName(CrossPhase p) {
  switch (p) {
    case CROSS_SEEK_RIDGE:     return "SEEK";
    case CROSS_PASS_RIDGE:     return "PASS";
    case CROSS_CENTER_ADVANCE: return "CENT";
    default:                   return "?";
  }
}
