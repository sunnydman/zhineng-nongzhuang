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
  const uint16_t slowMm = forward ? params_.frontSlowMm : (params_.rearStopMm + 190U);

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

  // 进入减速区后线性降速至 slowPwm，减小停机惯性滑行误差
  int16_t speedMag = params_.basePwm;
  if (lead.valid && lead.filteredMm < slowMm && slowMm > stopMm) {
    int32_t span = static_cast<int32_t>(slowMm - stopMm);
    int32_t rem = (lead.filteredMm > stopMm) ? static_cast<int32_t>(lead.filteredMm - stopMm) : 0;
    speedMag = static_cast<int16_t>(
        params_.slowPwm + (static_cast<int32_t>(params_.basePwm - params_.slowPwm) * rem) / span);
  }

  // --------------------------------------------------------------------------
  // 2. 左右垄壁检测与横向偏差计算
  //    约定：errMm > 0 表示左侧空间更大（车体偏右），前向需向左纠偏
  // --------------------------------------------------------------------------
  const bool leftValid = l.valid && (l.filteredMm <= params_.sideWallMaxMm);
  const bool rightValid = r.valid && (r.filteredMm <= params_.sideWallMaxMm);

  WallMode wallMode = WALL_NONE;
  int16_t errMm = 0;
  if (leftValid && rightValid) {
    // 双侧都有垄壁/围栏：以左右等距为中线（自动适应 ±5% 沟宽公差）
    wallMode = WALL_BOTH;
    errMm = static_cast<int16_t>(l.filteredMm) - static_cast<int16_t>(r.filteredMm);
  } else if (leftValid) {
    // 仅左侧有墙（例如刚过短垄 R4 开口）：贴左墙保持 singleWallMm(85mm)
    wallMode = WALL_LEFT_ONLY;
    errMm = static_cast<int16_t>(l.filteredMm) - static_cast<int16_t>(params_.singleWallMm);
  } else if (rightValid) {
    // 仅右侧有墙：贴右墙保持 singleWallMm(85mm)
    wallMode = WALL_RIGHT_ONLY;
    errMm = static_cast<int16_t>(params_.singleWallMm) - static_cast<int16_t>(r.filteredMm);
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

  int16_t usSteer = static_cast<int16_t>(
      params_.kp * static_cast<float>(errMm) + params_.kd * filteredDErr_);

  // --------------------------------------------------------------------------
  // 4. IMU 航向保持环（与超声横向居中叠加，防止斜角和画龙）
  // --------------------------------------------------------------------------
  float yawErrDeg = 0.0f;
  int16_t yawSteer = 0;
  const bool imuActive = useHeadingHold_ && (imu_ != nullptr) && imu_->isOnline(nowMs);
  if (imuActive) {
    yawErrDeg = ImuSerial::wrapAngleDeg(targetYawDeg_ - imu_->yawDeg());
    yawSteer = static_cast<int16_t>(params_.kpYawHold * yawErrDeg);
  }

  // 差速合成规则：
  // - 超声居中：前进时偏右(err>0)需左转(-usSteer)；倒车时偏右需车尾向左收(+usSteer)
  // - 航向保持：无论前进还是倒车，目标角在左(yawErr>0)均需车体逆时针转(-yawSteer)
  int16_t cmdForward = forward ? speedMag : -speedMag;
  int16_t cmdTurn = (forward ? -usSteer : usSteer) - yawSteer;
  cmdTurn = constrain(cmdTurn, -params_.maxSteerPwm, params_.maxSteerPwm);

  status_.wallMode = wallMode;
  status_.errMm = errMm;
  status_.yawErrDeg = yawErrDeg;
  status_.headingLocked = imuActive;
  status_.cmdForward = cmdForward;
  status_.cmdTurn = cmdTurn;

  motors_->setTank(cmdForward, cmdTurn);
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
