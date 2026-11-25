#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

// ============================================================================
// 三路舵机控制器（D32 / D33 / D34）
// 特点：
//   1. 上电默认不主动输出 PWM（不猛甩机械臂），收到指令后再 attach 驱动；
//   2. 支持逐个小幅度非阻塞摆动测试（90° -> 65° -> 115° -> 90°），便于安全核对引脚与转向；
//   3. 支持单路/全路指定角度与一键断力释放（detachAll）。
// ============================================================================
class ServoController {
 public:
  void begin();

  // 将引脚号 (32/33/34) 或索引 (0/1/2) 转换为内部通道索引，无效返回 -1
  static int8_t resolveIndex(int16_t pinOrIndex);
  static uint8_t pinOf(uint8_t idx);

  // 设置单个舵机角度 (0..180°)
  void setAngle(uint8_t idx, uint8_t deg);
  // 设置全部舵机到同一角度 (0..180°)
  void setAll(uint8_t deg);
  // 停止测试并释放全部舵机 PWM 信号（断力保护）
  void detachAll();

  // 启动非阻塞小幅摆动测试：targetIdx < 0 表示依次逐个测试 D32 -> D33 -> D34
  void startTestSweep(int8_t targetIdx, uint32_t nowMs);
  // 启动持续往复摆动测试（脱机测 6V 供电用：20° <-> 160°，每 1200ms 换向一次）
  void startLoopTest(uint8_t idx, uint32_t nowMs);
  void stopTest();
  void update(uint32_t nowMs);

  bool isTesting() const { return testing_; }
  bool isAttached(uint8_t idx) const;
  uint8_t currentAngle(uint8_t idx) const;

 private:
  static const uint8_t kPins[SERVO_COUNT];

  Servo servos_[SERVO_COUNT];
  bool attached_[SERVO_COUNT] = {false, false, false};
  uint8_t angles_[SERVO_COUNT] = {SERVO_CENTER_DEG, SERVO_CENTER_DEG, SERVO_CENTER_DEG};

  // 非阻塞测试状态机
  bool testing_ = false;
  bool testAll_ = false;
  bool loopMode_ = false;
  uint8_t testCh_ = 0;
  uint8_t testStep_ = 0;
  uint32_t testStepStartMs_ = 0;
};
