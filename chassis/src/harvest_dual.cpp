#include "harvest_dual.h"
#include "harvest_config.h"

void HarvestDual::update(const HarvestFrame& left, const HarvestFrame& right, float odomMm,
                         bool mayStrike, uint32_t nowMs) {
  const HarvestFrame* frames[] = {&left, &right};
  bool fresh[2] = {};
  bool newFrame[2] = {};
  bool ready[2] = {};
  slow_ = false;
  for (uint8_t i = 0; i < 2; ++i) {
    auto& side = sides_[i];
    const auto& frame = *frames[i];
    newFrame[i] = frame.received && !frame.stopMode && (!side.seen || frame.seq != side.seq) &&
                  nowMs - frame.rxMs <= HARVEST_FRAME_FRESH_MS;
    if (newFrame[i]) {
      if (!side.seen || static_cast<uint16_t>(frame.seq - side.seq) != 1 ||
          frame.rxMs - side.rxMs > HARVEST_FRAME_FRESH_MS) {
        side.positive = 0;
        side.clear = 0;
      }
      side.seen = true;
      side.seq = frame.seq;
      side.rxMs = frame.rxMs;
      if (frame.nearFruit) {
        side.occupied = true;
        side.lastNearMm = odomMm;
        side.clear = 0;
        if (side.positive < HARVEST_NEAR_FRAMES) ++side.positive;
      } else {
        side.positive = 0;
        if (side.clear < HARVEST_CLEAR_FRAMES) ++side.clear;
        if (side.clear >= HARVEST_CLEAR_FRAMES &&
            (!side.occupied || odomMm - side.lastNearMm >= HARVEST_CLEAR_TRAVEL_MM)) {
          side.occupied = false;
          side.served = false;
        }
      }
    }
    fresh[i] = !frame.stopMode && side.seen && nowMs - side.rxMs <= HARVEST_FRAME_FRESH_MS;
    if (!fresh[i]) {
      side.positive = 0;
      side.clear = 0;
    }
    if (side.retracting && nowMs - side.strikeMs >= HARVEST_STRIKE_CYCLE_MS) {
      side.retracting = false;
    }
    // 两侧各自确认清场，任何一侧丢帧或处于回收周期都保持慢行。
    slow_ |= !fresh[i] || side.clear < HARVEST_CLEAR_FRAMES || side.occupied || side.retracting;
    ready[i] = newFrame[i] && frame.nearFruit && frame.aligned && !side.served && !side.retracting &&
               side.positive >= HARVEST_NEAR_FRAMES && (frame.target == 'H' || frame.target == 'L');
  }

  if (!mayStrike || !fresh[0] || !fresh[1] ||
      (hasStrike_ && nowMs - lastStrikeMs_ < HARVEST_START_GAP_MS)) {
    return;
  }
  for (uint8_t offset = 0; offset < 2; ++offset) {
    const uint8_t i = (priority_ + offset) % 2;
    if (ready[i]) {
      auto& side = sides_[i];
      side.served = true;
      side.retracting = true;
      side.strikeMs = nowMs;
      side.action = frames[i]->target;
      hasStrike_ = true;
      lastStrikeMs_ = nowMs;
      priority_ = 1 - i;
      return;
    }
  }
}

char HarvestDual::takeStrike(uint8_t side) {
  const char action = sides_[side].action;
  sides_[side].action = 'N';
  return action;
}
