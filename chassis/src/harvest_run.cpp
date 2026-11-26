#include "harvest_config.h"

void HarvestRun::enter(State state, uint32_t nowMs, const char* event) {
  state_ = state;
  stateSinceMs_ = nowMs;
  event_ = event;
}

int16_t HarvestRun::speedCap(int16_t scanPwm) const {
  return state_ == SCAN ? scanPwm : -1;
}

void HarvestRun::advanceStation() {
  state_ = OUTSIDE;
  haveFrame_ = false;
  nearCount_ = 0;
  stoppedAlignedCount_ = 0;
  target_ = 'N';
}

void HarvestRun::update(const HarvestStation& station, bool inLane, uint8_t lane, float odomMm,
                        float maxWheelSpeed, bool mayStrike, const HarvestFrame& frame, uint32_t nowMs) {
  if (!inLane || lane == 0) {
    *this = HarvestRun{};
    return;
  }
  if (lane != lane_) {
    *this = HarvestRun{};
    lane_ = lane;
    originMm_ = odomMm;
  }
  laneDistanceMm_ = odomMm - originMm_;
  if (!station.valid()) {
    return;
  }
  if (state_ == DONE) {
    return;
  }
  if (state_ == STRIKE) {
    if (nowMs - stateSinceMs_ >= HARVEST_STRIKE_CYCLE_MS) {
      enter(DONE, nowMs, "retract_time_elapsed");
    }
    return;
  }
  if (laneDistanceMm_ >= station.endMm) {
    enter(DONE, nowMs, state_ == OUTSIDE || state_ == APPROACH ? "window_missed" : "passed_tree");
    return;
  }
  if (state_ == OUTSIDE) {
    if (laneDistanceMm_ < station.startMm) {
      return;
    }
    // 进入里程窗后才计新帧，窗外的背景果不能触发动作。
    haveFrame_ = frame.received;
    lastSeq_ = frame.seq;
    enter(APPROACH, nowMs, "entered_tree_window");
    return;
  }
  const bool fresh = frame.received && frame.stopMode && nowMs - frame.rxMs <= HARVEST_FRAME_FRESH_MS;
  const bool newFrame = fresh && (!haveFrame_ || frame.seq != lastSeq_);
  const bool consecutive = haveFrame_ && static_cast<uint16_t>(frame.seq - lastSeq_) == 1;
  if (!fresh || (newFrame && !consecutive)) {
    nearCount_ = 0;
    stoppedAlignedCount_ = 0;
  }
  if (newFrame) {
    haveFrame_ = true;
    lastSeq_ = frame.seq;
    nearCount_ = frame.nearFruit ? (nearCount_ < HARVEST_NEAR_FRAMES ? nearCount_ + 1 : nearCount_) : 0;
  }

  if (state_ == APPROACH) {
    if ((fresh && frame.nearFruit) || laneDistanceMm_ >= station.slowByMm) {
      enter(SCAN, nowMs, fresh && frame.nearFruit ? "tree_seen" : "surveyed_slow_point");
    }
    return;
  }
  const bool targetAligned = fresh && frame.nearFruit && frame.aligned &&
                             (frame.target == 'H' || frame.target == 'L');
  if (state_ == SETTLE) {
    if (!mayStrike || !fresh || nowMs - stateSinceMs_ >= HARVEST_SETTLE_MAX_MS) {
      enter(DONE, nowMs, "stop_recheck_rejected");
      return;
    }
    if (maxWheelSpeed > HARVEST_STOPPED_WHEEL_MMS || !targetAligned || frame.target != target_) {
      stoppedAlignedCount_ = 0;
    } else if (newFrame && static_cast<int32_t>(frame.rxMs - stateSinceMs_) >=
                               static_cast<int32_t>(HARVEST_SETTLE_MIN_MS) &&
               ++stoppedAlignedCount_ >= HARVEST_STOPPED_ALIGNED_FRAMES) {
      enter(STRIKE, nowMs, "stopped_and_reconfirmed");
    }
  } else if (nowMs - stateSinceMs_ >= HARVEST_SCAN_MAX_MS) {
    enter(DONE, nowMs, "scan_timeout");
  } else if (mayStrike && newFrame && targetAligned && nearCount_ >= HARVEST_NEAR_FRAMES) {
    target_ = frame.target;
    enter(SETTLE, nowMs, "axis_aligned_brake");
  }
}

const char* HarvestRun::stateName(State state) {
  switch (state) {
    case OUTSIDE: return "OUTSIDE";
    case APPROACH: return "APPROACH";
    case SCAN: return "SCAN";
    case SETTLE: return "SETTLE";
    case STRIKE: return "STRIKE";
    case DONE: return "DONE";
  }
  return "?";
}
