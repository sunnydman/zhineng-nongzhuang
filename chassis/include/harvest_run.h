#pragma once

#include <stdint.h>
#include <stddef.h>

struct HarvestFrame {
  bool received = false;
  uint16_t seq = 0;
  uint32_t rxMs = 0;
  bool nearFruit = false;
  char target = 'N';  // N=无许可，H=高果，L=低果。
  bool aligned = false;
  bool stopMode = false;  // HV2：果心对的是停车作业中线。
};

struct HarvestStation {
  float startMm;
  float slowByMm;
  float endMm;
  constexpr bool valid() const { return startMm >= 0 && startMm <= slowByMm && slowByMm < endMm; }
};

// 单工位停车疏果。以本沟开始后的前向里程为基准。
class HarvestRun {
 public:
  enum State : uint8_t { OUTSIDE, APPROACH, SCAN, SETTLE, STRIKE, DONE };

  void update(const HarvestStation& station, bool inLane, uint8_t lane, float odomMm,
              float maxWheelSpeed, bool mayStrike, const HarvestFrame& frame, uint32_t nowMs);
  void advanceStation();
  bool holding() const { return state_ == SETTLE || state_ == STRIKE; }
  State state() const { return state_; }
  char target() const { return target_; }
  const char* event() const { return event_; }
  float laneDistanceMm() const { return laneDistanceMm_; }
  int16_t speedCap(int16_t scanPwm) const;
  static const char* stateName(State state);

 private:
  void enter(State state, uint32_t nowMs, const char* event);

  State state_ = OUTSIDE;
  uint8_t lane_ = 0;
  float originMm_ = 0;
  float laneDistanceMm_ = 0;
  uint32_t stateSinceMs_ = 0;
  bool haveFrame_ = false;
  uint16_t lastSeq_ = 0;
  uint8_t nearCount_ = 0;
  uint8_t stoppedAlignedCount_ = 0;
  char target_ = 'N';
  const char* event_ = "";
};
