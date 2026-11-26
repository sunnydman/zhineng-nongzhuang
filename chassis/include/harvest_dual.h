#pragma once

#include "harvest_run.h"

class HarvestDual {
 public:
  void update(const HarvestFrame& left, const HarvestFrame& right, float odomMm,
              bool mayStrike, uint32_t nowMs);
  char takeStrike(uint8_t side);
  bool slow() const { return slow_; }
  bool occupied(uint8_t side) const { return sides_[side].occupied; }

 private:
  struct Side {
    bool seen = false;
    uint16_t seq = 0;
    uint32_t rxMs = 0;
    bool occupied = false;
    bool served = false;
    uint8_t positive = 0;
    uint8_t clear = 0;
    float lastNearMm = 0;
    char action = 'N';
    bool retracting = false;
    uint32_t strikeMs = 0;
  };

  Side sides_[2];
  bool slow_ = true;
  bool hasStrike_ = false;
  uint32_t lastStrikeMs_ = 0;
  uint8_t priority_ = 0;
};
