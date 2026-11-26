// ============================================================================
// 【文件作用】垄沟内居中循迹控制器实现（直行/倒车走正 + 到端头自动减速稳停）
//   - 双侧都有墙时按 L=R 居中，单侧有墙（如短垄缺口）时贴单侧墙保持目标距离，
//     叠加串级 ±5° 偏航角限幅与陀螺仪锁直，防止高速画龙或撞壁。
// ============================================================================
#include "tracker.h"

// ============================================================================
// 垄沟居中循迹器实现：
//   - 航向保持（IMU 在线时锁死目标角度）+ 左右超声居中 PD 双环融合
//   - 行进方向端头（前/后超声）具备：入口时间屏蔽 + 连续 2 帧防抖确认 + 线性减速停车
//   - 调试技巧：串口打开 'U' 可实时查看 [模式|墙态] err=... 和左右输出 L/R
// ============================================================================

void CorridorTracker::begin(MotorDriver* motors, UltrasonicArray* sensors, ImuSerial* imu) {
  motors_ = motors;
  sensors_ = sensors;
  imu_ = imu;
  stop();
}

void CorridorTracker::stop() {
  status_.mode = MODE_IDLE;
  status_.wallMode = WALL_NONE;
  status_.errMm = 0;
  status_.yawErrDeg = 0.0f;
  status_.headingLocked = false;
  status_.cmdForward = 0;
  status_.cmdTurn = 0;
  useHeadingHold_ = false;
  resetController();
  if (motors_ != nullptr) {
    motors_->stopAll();
  }
}

void CorridorTracker::startForward(uint32_t nowMs) {
  resetController();
  trackStartMs_ = nowMs;
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    targetYawDeg_ = imu_->yawDeg();
    useHeadingHold_ = true;
  } else {
    useHeadingHold_ = false;
  }
  status_.mode = MODE_TRACK_FWD;
}

void CorridorTracker::startForwardWithHeading(float targetYawDeg) {
  resetController();
  trackStartMs_ = millis();
  targetYawDeg_ = ImuSerial::wrapAngleDeg(targetYawDeg);
  useHeadingHold_ = (imu_ != nullptr);
  status_.mode = MODE_TRACK_FWD;
}

void CorridorTracker::startReverse(uint32_t nowMs) {
  resetController();
  trackStartMs_ = nowMs;
  if (imu_ != nullptr && imu_->isOnline(nowMs)) {
    targetYawDeg_ = imu_->yawDeg();
    useHeadingHold_ = true;
  } else {
    useHeadingHold_ = false;
  }
  status_.mode = MODE_TRACK_REV;
}

void CorridorTracker::startReverseWithHeading(float targetYawDeg) {
  resetController();
  trackStartMs_ = millis();
  targetYawDeg_ = ImuSerial::wrapAngleDeg(targetYawDeg);
  useHeadingHold_ = (imu_ != nullptr);
  status_.mode = MODE_TRACK_REV;
}

void CorridorTracker::startManualTest(uint32_t nowMs, uint32_t durationMs) {
  resetController();
  useHeadingHold_ = false;
  status_.mode = MODE_MANUAL_TEST;
  testDeadlineMs_ = nowMs + durationMs;
}

void CorridorTracker::resetController() {
  prevWallMode_ = WALL_NONE;
  prevErrValid_ = false;
  prevErrMm_ = 0;
  lastSideUpdateMs_ = 0;
  filteredDErr_ = 0.0f;
  endStopStreak_ = 0;
  lastLeadUpdateMs_ = 0;
  smoothSpeedPwm_ = 0.0f;
  lastSpeedRampMs_ = 0;
  singleWallOverrideMm_ = 0;
}

int16_t CorridorTracker::rampSpeed(int16_t targetPwm, uint32_t nowMs) {
  if (lastSpeedRampMs_ == 0) {
    lastSpeedRampMs_ = nowMs;
    smoothSpeedPwm_ = static_cast<float>(min(targetPwm, DRIVE_SOFT_START_PWM));
    return static_cast<int16_t>(smoothSpeedPwm_);
  }
  uint32_t dtMs = nowMs - lastSpeedRampMs_;
  if (dtMs > 0) {
    lastSpeedRampMs_ = nowMs;
    float dtSec = min(static_cast<float>(dtMs) * 0.001f, 0.10f);
    float target = static_cast<float>(targetPwm);
    if (target > smoothSpeedPwm_) {
      smoothSpeedPwm_ = min(target, smoothSpeedPwm_ + SPEED_ACCEL_PWM_PER_SEC * dtSec);
    } else if (target < smoothSpeedPwm_) {
      smoothSpeedPwm_ = max(target, smoothSpeedPwm_ - SPEED_DECEL_PWM_PER_SEC * dtSec);
    }
  }
  return static_cast<int16_t>(smoothSpeedPwm_);
}

void CorridorTracker::update(uint32_t nowMs) {
  if (motors_ == nullptr || sensors_ == nullptr) {
    return;
  }

  switch (status_.mode) {
    case MODE_IDLE:
    case MODE_STOPPED_END:
      return;

    case MODE_MANUAL_TEST:
      if (static_cast<int32_t>(nowMs - testDeadlineMs_) >= 0) {
        stop();
        Serial.println(F("[TEST] 测试时间到 -> 自动停机"));
      }
      return;

    case MODE_TRACK_FWD:
      runTracking(nowMs, true);
      return;

    case MODE_TRACK_REV:
      runTracking(nowMs, false);
      return;
  }
}

void CorridorTracker::runTracking(uint32_t nowMs, bool forward) {
  if (!sensors_->allSampledOnce()) {
    motors_->stopAll();
    return;
  }

  const auto& f = sensors_->get(US_FRONT);
  const auto& b = sensors_->get(US_REAR);
  const auto& l = sensors_->get(US_LEFT);
  const auto& r = sensors_->get(US_RIGHT);

  // --------------------------------------------------------------------------
  // 1. 端头围栏减速与停车判定（带入口屏蔽 + 连续 2 帧防抖，杜绝单帧杂波误停）
  // --------------------------------------------------------------------------
  const auto& lead = forward ? f : b;
  const uint16_t stopMm = forward ? params_.frontStopMm : params_.rearStopMm;
  const uint16_t slowMm = forward ? params_.frontSlowMm : (params_.rearStopMm + 320U);

  if ((nowMs - trackStartMs_) >= LANE_END_GUARD_MS && lead.lastUpdateMs != lastLeadUpdateMs_) {
    lastLeadUpdateMs_ = lead.lastUpdateMs;
    if (lead.valid && lead.filteredMm <= stopMm) {
      if (endStopStreak_ < 255) {
        endStopStreak_++;
      }
    } else {
      endStopStreak_ = 0;
    }
  }

  if (endStopStreak_ >= END_STOP_CONFIRM_COUNT) {
    motors_->stopAll();
    status_.mode = MODE_STOPPED_END;
    status_.cmdForward = 0;
    status_.cmdTurn = 0;
    resetController();
    Serial.print(F("[TRACK] 到达"));
    Serial.print(forward ? F("前方") : F("后方"));
    Serial.print(F("停机距离: "));
    Serial.print(lead.filteredMm);
    Serial.println(F(" mm -> 已停稳"));
    return;
  }

  // 进入减速区后线性降速至 slowPwm，减小停机惯性滑行误差（倒车采用专用稳速，防止高速甩尾画龙）
  const int16_t cruisePwm = forward ? params_.basePwm : min(params_.basePwm, DEFAULT_REVERSE_PWM);
  int16_t speedMag = cruisePwm;
  if (lead.valid && lead.filteredMm < slowMm && slowMm > stopMm) {
    int32_t span = static_cast<int32_t>(slowMm - stopMm);
    int32_t rem = (lead.filteredMm > stopMm) ? static_cast<int32_t>(lead.filteredMm - stopMm) : 0;
    speedMag = static_cast<int16_t>(
        params_.slowPwm + (static_cast<int32_t>(cruisePwm - params_.slowPwm) * rem) / span);
  }

  // --------------------------------------------------------------------------
  // 2. 左右垄壁检测与横向偏差计算
  //    约定：errMm > 0 表示左侧空间更大（车体偏右），前向需向左纠偏
  // --------------------------------------------------------------------------
  const bool leftValid = l.valid && (l.filteredMm <= params_.sideWallMaxMm);
  const bool rightValid = r.valid && (r.filteredMm <= params_.sideWallMaxMm);

  WallMode wallMode = WALL_NONE;
  int16_t errMm = 0;
  const uint16_t singleWallTargetMm =
      (singleWallOverrideMm_ > 0) ? singleWallOverrideMm_ : params_.singleWallMm;
  if (leftValid && rightValid) {
    // 双侧都有垄壁/围栏：以左右等距为中线（自动适应 ±5% 沟宽公差）
    wallMode = WALL_BOTH;
    errMm = static_cast<int16_t>(l.filteredMm) - static_cast<int16_t>(r.filteredMm);
  } else if (leftValid) {
    // 仅左侧有墙（如第 1 垄末端或短垄 R4 开口）：贴左墙保持目标距离
    wallMode = WALL_LEFT_ONLY;
    errMm = static_cast<int16_t>(l.filteredMm) - static_cast<int16_t>(singleWallTargetMm);
  } else if (rightValid) {
    // 仅右侧有墙：贴右墙保持目标距离
    wallMode = WALL_RIGHT_ONLY;
    errMm = static_cast<int16_t>(singleWallTargetMm) - static_cast<int16_t>(r.filteredMm);
  } else {
    // 两侧均开阔：仅靠 IMU 锁直并低速通过
    wallMode = WALL_NONE;
    errMm = 0;
    speedMag = min(speedMag, params_.slowPwm);
  }

  if (abs(errMm) <= params_.errDeadbandMm) {
    errMm = 0;
  }

  // 双墙/单墙/开阔切换时清零微分历史，避免模式跳变引起车头猛甩
  if (wallMode != prevWallMode_) {
    prevWallMode_ = wallMode;
    prevErrValid_ = false;
    filteredDErr_ = 0.0f;
    lastSideUpdateMs_ = 0;
  }

  // --------------------------------------------------------------------------
  // 3. 横向居中 PD 计算（仅在左右超声真正刷新时更新微分并做一阶平滑）
  // --------------------------------------------------------------------------
  const uint32_t sideUpdateMs = (l.lastUpdateMs >= r.lastUpdateMs) ? l.lastUpdateMs : r.lastUpdateMs;
  if (wallMode == WALL_NONE) {
    filteredDErr_ = 0.0f;
    prevErrValid_ = false;
    lastSideUpdateMs_ = sideUpdateMs;
  } else if (sideUpdateMs != lastSideUpdateMs_) {
    if (prevErrValid_ && lastSideUpdateMs_ != 0 && sideUpdateMs > lastSideUpdateMs_) {
      float dtSec = static_cast<float>(sideUpdateMs - lastSideUpdateMs_) * 0.001f;
      if (dtSec >= 0.02f && dtSec <= 0.50f) {
        float rawDErr = static_cast<float>(errMm - prevErrMm_) / dtSec;
        filteredDErr_ = 0.5f * rawDErr + 0.5f * filteredDErr_;
      }
    } else {
      filteredDErr_ = 0.0f;
    }
    prevErrMm_ = errMm;
    prevErrValid_ = true;
    lastSideUpdateMs_ = sideUpdateMs;
  }

  // 统一平滑加减速斜率限制（消除起步、开阔区恢复巡航与超声阶梯减速时的顿挫蹿车）
  speedMag = rampSpeed(speedMag, nowMs);

  // --------------------------------------------------------------------------
  // 4. 串级偏航角限幅 (±5°) + 轻度麦轮微横移辅助（IMU 离线时自动回退传统 PD 差速）
  //    约定：errMm > 0 车体偏右，前进需向左切 (yawOffset > 0)、倒车车头需向右偏 (yawOffset < 0)
  // --------------------------------------------------------------------------
  float yawErrDeg = 0.0f;
  int16_t cmdForward = forward ? speedMag : -speedMag;
  int16_t cmdTurn = 0;
  int16_t cmdStrafe = 0;
  const bool imuActive = useHeadingHold_ && (imu_ != nullptr) && imu_->isOnline(nowMs);

  if (imuActive) {
    float rawYawOff = 0.0f;
    if (wallMode != WALL_NONE) {
      // 倒车时以 IMU 死锁直线为主，横向超声外环切入角收紧至 ±2.5°、增益减半，避免倒车甩尾画龙
      const float kpCascade = forward ? CASCADE_KP_DEG_PER_MM : (CASCADE_KP_DEG_PER_MM * 0.5f);
      const float maxYawOff = forward ? CASCADE_MAX_YAW_DEG : (CASCADE_MAX_YAW_DEG * 0.5f);
      rawYawOff = kpCascade * static_cast<float>(errMm) +
                  CASCADE_KD_DEG_PER_MMS * filteredDErr_;
      rawYawOff = constrain(rawYawOff, -maxYawOff, maxYawOff);
      cmdStrafe = constrain(
          -static_cast<int16_t>(DRIVE_STRAFE_KP * static_cast<float>(errMm)),
          -DRIVE_STRAFE_MAX_PWM,
          DRIVE_STRAFE_MAX_PWM);
    }
    float signedYawOff = forward ? rawYawOff : -rawYawOff;
    float desiredYaw = ImuSerial::wrapAngleDeg(targetYawDeg_ + signedYawOff);
    yawErrDeg = ImuSerial::wrapAngleDeg(desiredYaw - imu_->yawDeg());
    const float yawKp = forward ? params_.kpYawHold : (params_.kpYawHold * 1.15f);
    cmdTurn = -static_cast<int16_t>(yawKp * yawErrDeg);
    cmdTurn = constrain(cmdTurn, -params_.maxSteerPwm, params_.maxSteerPwm);
    motors_->setMecanum(cmdForward, cmdStrafe, cmdTurn);
  } else {
    int16_t usSteer = static_cast<int16_t>(
        params_.kp * static_cast<float>(errMm) + params_.kd * filteredDErr_);
    cmdTurn = constrain(forward ? -usSteer : usSteer, -params_.maxSteerPwm, params_.maxSteerPwm);
    motors_->setTank(cmdForward, cmdTurn);
  }

  status_.wallMode = wallMode;
  status_.errMm = errMm;
  status_.yawErrDeg = yawErrDeg;
  status_.headingLocked = imuActive;
  status_.cmdForward = cmdForward;
  status_.cmdTurn = cmdTurn;
}

const char* CorridorTracker::modeName(TrackerMode mode) {
  switch (mode) {
    case MODE_IDLE:        return "IDLE";
    case MODE_TRACK_FWD:   return "TRACK_FWD";
    case MODE_TRACK_REV:   return "TRACK_REV";
    case MODE_STOPPED_END: return "STOP_END";
    case MODE_MANUAL_TEST: return "TEST";
    default:               return "UNKNOWN";
  }
}

const char* CorridorTracker::wallModeName(WallMode mode) {
  switch (mode) {
    case WALL_NONE:       return "OPEN";
    case WALL_BOTH:       return "BOTH";
    case WALL_LEFT_ONLY:  return "LEFT";
    case WALL_RIGHT_ONLY: return "RIGHT";
    default:              return "?";
  }
}
