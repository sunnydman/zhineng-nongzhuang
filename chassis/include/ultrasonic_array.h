// ============================================================================
// 【文件作用】四路超声波测距头文件（声明接口，平时调参不用改）
//   - 负责前(F)/后(B)/左(L)/右(R) 四路 HC-SR04 防串扰轮询测距与滤波。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

enum UltrasonicId : uint8_t {
  US_FRONT = 0,
  US_REAR  = 1,
  US_LEFT  = 2,
  US_RIGHT = 3,
  US_COUNT = 4
};

struct UltrasonicReading {
  uint16_t rawMm = 0;
  uint16_t filteredMm = 0;
  bool valid = false;
  uint32_t lastUpdateMs = 0;
};

class UltrasonicArray {
 public:
  void begin();

  // 清空四路中值滤波历史（用于 90° 转向停稳后，防止转弯前的近墙残留值误触发）
  void clearFilters();

  // 分时轮询更新：每间隔 US_SLOT_INTERVAL_MS 触发一路，按 前->右->后->左 交替降低串扰
  // 返回本次调用是否完成了一路新采样
  bool update(uint32_t nowMs);

  const UltrasonicReading& get(UltrasonicId id) const;
  bool allSampledOnce() const;
  UltrasonicId lastSampledId() const { return lastSampled_; }
  static const char* sensorName(UltrasonicId id);

 private:
  struct SensorPins {
    uint8_t trigPin;
    uint8_t echoPin;
  };

  struct FilterState {
    uint16_t window[3] = {0, 0, 0};
    uint8_t count = 0;
    uint8_t head = 0;
    uint8_t missStreak = 0;
    bool sampledOnce = false;
  };

  uint16_t measureOnceMm(UltrasonicId id) const;
  static uint16_t median3(uint16_t a, uint16_t b, uint16_t c);

  static const SensorPins kPins[US_COUNT];
  static const UltrasonicId kPollOrder[US_COUNT];

  UltrasonicReading readings_[US_COUNT];
  FilterState filters_[US_COUNT];
  uint8_t pollIndex_ = 0;
  uint32_t lastSlotMs_ = 0;
  UltrasonicId lastSampled_ = US_FRONT;
};
