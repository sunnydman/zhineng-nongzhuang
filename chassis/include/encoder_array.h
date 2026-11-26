// ============================================================================
// 【文件作用】四路正交编码器头文件（声明接口，平时调参不用改）
//   - 负责读取四个车轮的转速 (mm/s) 与行驶里程 (mm)。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"
#include "motor_driver.h"

// 四路 AB 相正交编码器驱动（使用 Mega 2560 Port K / PCINT2：A8-A15，4 倍频解码）
// 180° 掉头后实车通道映射：
//   WHEEL_RL (新左后): A8(PK0),  A9(PK1)
//   WHEEL_RR (新右后): A10(PK2), A11(PK3)
//   WHEEL_FL (新左前): A12(PK4), A13(PK5)
//   WHEEL_FR (新右前): A14(PK6), A15(PK7)
class EncoderArray {
 public:
  void begin();
  void update(uint32_t nowMs);
  void reset();

  int32_t counts(WheelId wheel) const;
  float distanceMm(WheelId wheel) const;
  float speedMmPerSec(WheelId wheel) const;

  float avgDistanceMm() const;
  float avgSpeedMmPerSec() const;
  bool anyActivity() const;

  // 底层硬件诊断：读取 A8-A15 (bit0..7) 的实时电平与逐脚原始边沿计数
  uint8_t rawPink() const;
  uint32_t rawPinEdges(uint8_t bitIndex) const;

 private:
  int32_t snapshotCounts_[WHEEL_COUNT] = {0, 0, 0, 0};
  int32_t prevCounts_[WHEEL_COUNT] = {0, 0, 0, 0};
  float speedMmPerSec_[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint32_t lastSampleMs_ = 0;

  static float mmPerCount();
};
