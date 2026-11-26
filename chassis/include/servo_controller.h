// ============================================================================
// 【文件作用】四路疏果击打舵机控制器头文件（声明接口，平时调参不用改）
//   - 负责 LD(左低)/RD(右低)/LU(左高)/RU(右高) 收回、外扩击打与角度标定接口。
// ============================================================================
#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

// ============================================================================
// 四路击打舵机控制器（D32 左低 / D33 右低 / D34 左高 / D35 右高）
// 已标定旋转方向（低位外扩 140° / 高位外扩 135°）：
//   LD (左低/D32): 收回 30°  -> 外扩 170° (+140°)
//   RD (右低/D33): 收回 150° -> 外扩 10°  (-140°)
//   LU (左高/D34): 收回 150° -> 外扩 15°  (-135°)
//   RU (右高/D35): 收回 30°  -> 外扩 165° (+135°)
// ============================================================================
enum ServoChannel : uint8_t {
  SERVO_LD = 0,  // 左低 (D32)
  SERVO_RD = 1,  // 右低 (D33)
  SERVO_LU = 2,  // 左高 (D34)
  SERVO_RU = 3   // 右高 (D35)
};

class ServoController {
 public:
  void begin();

  // 支持名称 (ld/rd/lu/ru) 或引脚号 (32/33/34/35) 或索引 (0..3) 解析通道，无效返回 -1
  static int8_t resolveIndex(int16_t pinOrIndex);
  static int8_t resolveName(const char* name);
  static uint8_t pinOf(uint8_t idx);
  static const char* nameOf(uint8_t idx);
  static uint8_t idleDegOf(uint8_t idx);
  static uint8_t strikeDegOf(uint8_t idx);

  // 设置单个舵机角度 (0..180°)
  void setAngle(uint8_t idx, uint8_t deg);
  // 错峰把全部 4 路归到统一角度（如 90° 装机调零位）
  void setAllStaggered(uint8_t deg, uint16_t gapMs = 250);

  // 语义化动作接口（自动适配各路已标定的旋转方向）
  void retract(uint8_t idx);                       // 单路贴身收回
  void retractAt(uint8_t idx, uint8_t angle);
  void retractAll(uint16_t gapMs = 200);           // 四路错峰全贴身收回
  void extend(uint8_t idx);                        // 单路外扩至击打位
  void triggerStrike(uint8_t idx, uint32_t nowMs); // 非阻塞单次击打（外扩 -> 保持 -> 自动收回）
  void triggerStrikeAt(uint8_t idx, uint8_t angle, uint8_t home, uint32_t nowMs);

  // 停止测试/击打并释放全部舵机 PWM 信号（断力保护）
  void detachAll();

  // 启动非阻塞方向标定测试：90° -> +25°(115°) -> 90° -> -25°(65°) -> 90°
  void startTestSweep(int8_t targetIdx, uint32_t nowMs);
  // 持续往复摆动测试（脱机测 6V 供电用）
  void startLoopTest(uint8_t idx, uint32_t nowMs);
  void stopTest();
  void update(uint32_t nowMs);

  bool isTesting() const { return testing_; }
  bool isAttached(uint8_t idx) const;
  uint8_t currentAngle(uint8_t idx) const;
  void printStatus() const;

 private:
  static const uint8_t kPins[SERVO_COUNT];
  static const char* const kNames[SERVO_COUNT];
  static const uint8_t kIdleDeg[SERVO_COUNT];
  static const uint8_t kStrikeDeg[SERVO_COUNT];

  Servo servos_[SERVO_COUNT];
  bool attached_[SERVO_COUNT] = {false, false, false, false};
  uint8_t angles_[SERVO_COUNT] = {
      SERVO_LD_IDLE_DEG, SERVO_RD_IDLE_DEG, SERVO_LU_IDLE_DEG, SERVO_RU_IDLE_DEG};

  // 每路独立的非阻塞击打收回定时（支持左右并行独立击打，互不阻塞）
  bool strikeActive_[SERVO_COUNT] = {false, false, false, false};
  uint32_t strikeRetractAtMs_[SERVO_COUNT] = {0, 0, 0, 0};
  uint8_t strikeHomeDeg_[SERVO_COUNT] = {};

  // 非阻塞标定测试状态机
  bool testing_ = false;
  bool testAll_ = false;
  bool loopMode_ = false;
  uint8_t testCh_ = 0;
  uint8_t testStep_ = 0;
  uint32_t testStepStartMs_ = 0;
};
