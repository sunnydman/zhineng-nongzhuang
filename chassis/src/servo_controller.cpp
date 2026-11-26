// ============================================================================
// 【文件作用】四路击打舵机控制器实现（LD=D32, RD=D33, LU=D34, RU=D35，Timer5）
//   - 负责四根疏果击打杆的错峰竖直收回 (HOME)、外扩击打 (OUT/HIT) 与自动收回；
//   - 如需微调各路竖直收回角或外扩击打角度，直接改 include/config.h 即可。
// ============================================================================
#include "servo_controller.h"

#include <string.h>

const uint8_t ServoController::kPins[SERVO_COUNT] = {
    PIN_SERVO_LD,  // 32: 左低
    PIN_SERVO_RD,  // 33: 右低
    PIN_SERVO_LU,  // 34: 左高
    PIN_SERVO_RU,  // 35: 右高
};

const char* const ServoController::kNames[SERVO_COUNT] = {
    "LD(左低/D32)",
    "RD(右低/D33)",
    "LU(左高/D34)",
    "RU(右高/D35)",
};

const uint8_t ServoController::kIdleDeg[SERVO_COUNT] = {
    SERVO_LD_IDLE_DEG,
    SERVO_RD_IDLE_DEG,
    SERVO_LU_IDLE_DEG,
    SERVO_RU_IDLE_DEG,
};

const uint8_t ServoController::kStrikeDeg[SERVO_COUNT] = {
    SERVO_LD_STRIKE_DEG,
    SERVO_RD_STRIKE_DEG,
    SERVO_LU_STRIKE_DEG,
    SERVO_RU_STRIKE_DEG,
};

void ServoController::begin() {
  // 上电保持不挂载 PWM，避免未知初始位置时舵机猛甩撞击机械限位
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    attached_[i] = false;
    angles_[i] = kIdleDeg[i];
    strikeActive_[i] = false;
    strikeRetractAtMs_[i] = 0;
  }
  testing_ = false;
  loopMode_ = false;
}

int8_t ServoController::resolveIndex(int16_t pinOrIndex) {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    if (pinOrIndex == static_cast<int16_t>(kPins[i])) {
      return static_cast<int8_t>(i);
    }
  }
  if (pinOrIndex >= 0 && pinOrIndex < SERVO_COUNT) {
    return static_cast<int8_t>(pinOrIndex);
  }
  return -1;
}

int8_t ServoController::resolveName(const char* name) {
  if (name == nullptr) {
    return -1;
  }
  if (strcmp(name, "ld") == 0) return SERVO_LD;
  if (strcmp(name, "rd") == 0) return SERVO_RD;
  if (strcmp(name, "lu") == 0) return SERVO_LU;
  if (strcmp(name, "ru") == 0) return SERVO_RU;
  if (name[0] >= '0' && name[0] <= '9') {
    return resolveIndex(static_cast<int16_t>(atoi(name)));
  }
  return -1;
}

uint8_t ServoController::pinOf(uint8_t idx) {
  return (idx < SERVO_COUNT) ? kPins[idx] : 0;
}

const char* ServoController::nameOf(uint8_t idx) {
  return (idx < SERVO_COUNT) ? kNames[idx] : "??";
}

uint8_t ServoController::idleDegOf(uint8_t idx) {
  return (idx < SERVO_COUNT) ? kIdleDeg[idx] : SERVO_CENTER_DEG;
}

uint8_t ServoController::strikeDegOf(uint8_t idx) {
  return (idx < SERVO_COUNT) ? kStrikeDeg[idx] : SERVO_CENTER_DEG;
}

void ServoController::setAngle(uint8_t idx, uint8_t deg) {
  if (idx >= SERVO_COUNT) {
    return;
  }
  if (deg > 180) {
    deg = 180;
  }
  if (!attached_[idx]) {
    servos_[idx].attach(kPins[idx], SERVO_MIN_US, SERVO_MAX_US);
    servos_[idx].write(deg);
    attached_[idx] = true;
  } else {
    servos_[idx].write(deg);
  }
  angles_[idx] = deg;
}

void ServoController::setAllStaggered(uint8_t deg, uint16_t gapMs) {
  stopTest();
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    Serial.print(F("[SERVO] 归位 "));
    Serial.print(kNames[i]);
    Serial.print(F(" -> "));
    Serial.print(deg);
    Serial.println(F(" deg"));
    setAngle(i, deg);
    if (gapMs > 0 && (i + 1) < SERVO_COUNT) {
      delay(gapMs);
    }
  }
}

void ServoController::retract(uint8_t idx) {
  if (idx < SERVO_COUNT) {
    retractAt(idx, kIdleDeg[idx]);
  }
}

void ServoController::retractAt(uint8_t idx, uint8_t angle) {
  if (idx >= SERVO_COUNT) {
    return;
  }
  strikeActive_[idx] = false;
  setAngle(idx, angle);
  Serial.print(F("[SERVO] 收回 "));
  Serial.print(kNames[idx]);
  Serial.print(F(" -> "));
  Serial.print(angle);
  Serial.println(F(" deg"));
}

void ServoController::retractAll(uint16_t gapMs) {
  stopTest();
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    retract(i);
    if (gapMs > 0 && (i + 1) < SERVO_COUNT) {
      delay(gapMs);
    }
  }
}

void ServoController::extend(uint8_t idx) {
  if (idx >= SERVO_COUNT) {
    return;
  }
  strikeActive_[idx] = false;
  setAngle(idx, kStrikeDeg[idx]);
  Serial.print(F("[SERVO] 外扩 "));
  Serial.print(kNames[idx]);
  Serial.print(F(" -> "));
  Serial.print(kStrikeDeg[idx]);
  Serial.println(F(" deg"));
}

void ServoController::triggerStrike(uint8_t idx, uint32_t nowMs) {
  if (idx < SERVO_COUNT) {
    triggerStrikeAt(idx, kStrikeDeg[idx], kIdleDeg[idx], nowMs);
  }
}

void ServoController::triggerStrikeAt(uint8_t idx, uint8_t angle, uint8_t home, uint32_t nowMs) {
  if (idx >= SERVO_COUNT) {
    return;
  }
  testing_ = false;
  loopMode_ = false;
  setAngle(idx, angle);
  strikeHomeDeg_[idx] = home;
  strikeActive_[idx] = true;
  strikeRetractAtMs_[idx] = nowMs + SERVO_STRIKE_HOLD_MS;
  Serial.print(F("[SERVO] 击打触发 "));
  Serial.print(kNames[idx]);
  Serial.print(F(" : "));
  Serial.print(home);
  Serial.print(F(" -> "));
  Serial.print(angle);
  Serial.println(F(" deg (自动收回)"));
}

void ServoController::detachAll() {
  testing_ = false;
  loopMode_ = false;
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    strikeActive_[i] = false;
    if (attached_[i]) {
      servos_[i].detach();
      attached_[i] = false;
    }
  }
}

void ServoController::startTestSweep(int8_t targetIdx, uint32_t nowMs) {
  testing_ = true;
  loopMode_ = false;
  testAll_ = (targetIdx < 0);
  testCh_ = testAll_ ? 0 : static_cast<uint8_t>(targetIdx);
  testStep_ = 0;
  testStepStartMs_ = nowMs;

  Serial.print(F("[SERVO] 方向标定 (90 -> +25°=115 -> 90 -> -25°=65 -> 90): "));
  Serial.println(kNames[testCh_]);
  setAngle(testCh_, SERVO_CENTER_DEG);
}

void ServoController::startLoopTest(uint8_t idx, uint32_t nowMs) {
  if (idx >= SERVO_COUNT) {
    return;
  }
  testing_ = true;
  loopMode_ = true;
  testAll_ = false;
  testCh_ = idx;
  testStep_ = 0;
  testStepStartMs_ = nowMs;

  Serial.print(F("[SERVO] 启动持续往复摆动测试 (20 <-> 160 deg, 周期 1.2s): "));
  Serial.println(kNames[testCh_]);
  setAngle(testCh_, 20);
}

void ServoController::stopTest() {
  testing_ = false;
  loopMode_ = false;
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    strikeActive_[i] = false;
  }
}

void ServoController::update(uint32_t nowMs) {
  // 1. 检查四路各自独立的击打自动收回定时
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    if (strikeActive_[i] && static_cast<int32_t>(nowMs - strikeRetractAtMs_[i]) >= 0) {
      strikeActive_[i] = false;
      setAngle(i, strikeHomeDeg_[i]);
      Serial.print(F("[SERVO] 击打完成自动收回 "));
      Serial.print(kNames[i]);
      Serial.print(F(" -> "));
      Serial.print(strikeHomeDeg_[i]);
      Serial.println(F(" deg"));
    }
  }

  if (!testing_) {
    return;
  }

  // 2. 脱机循环测试模式
  if (loopMode_) {
    if ((nowMs - testStepStartMs_) < 1200UL) {
      return;
    }
    testStepStartMs_ = nowMs;
    testStep_ = (testStep_ == 0) ? 1 : 0;
    setAngle(testCh_, (testStep_ == 0) ? 20 : 160);
    return;
  }

  // 3. 方向标定测试模式
  if ((nowMs - testStepStartMs_) < SERVO_TEST_STEP_MS) {
    return;
  }
  testStepStartMs_ = nowMs;
  testStep_++;

  if (testStep_ == 1) {
    uint8_t hi = SERVO_CENTER_DEG + SERVO_TEST_OFFSET;
    Serial.print(F("[SERVO] ")); Serial.print(kNames[testCh_]);
    Serial.print(F(" -> ")); Serial.print(hi); Serial.println(F(" deg  【+25°方向】"));
    setAngle(testCh_, hi);
  } else if (testStep_ == 2) {
    Serial.print(F("[SERVO] ")); Serial.print(kNames[testCh_]);
    Serial.print(F(" -> ")); Serial.print(SERVO_CENTER_DEG); Serial.println(F(" deg  (回中位)"));
    setAngle(testCh_, SERVO_CENTER_DEG);
  } else if (testStep_ == 3) {
    uint8_t lo = SERVO_CENTER_DEG - SERVO_TEST_OFFSET;
    Serial.print(F("[SERVO] ")); Serial.print(kNames[testCh_]);
    Serial.print(F(" -> ")); Serial.print(lo); Serial.println(F(" deg  【-25°方向】"));
    setAngle(testCh_, lo);
  } else if (testStep_ == 4) {
    Serial.print(F("[SERVO] ")); Serial.print(kNames[testCh_]);
    Serial.print(F(" -> ")); Serial.print(SERVO_CENTER_DEG); Serial.println(F(" deg  (归中完成)"));
    setAngle(testCh_, SERVO_CENTER_DEG);
  } else {
    if (testAll_ && (testCh_ + 1) < SERVO_COUNT) {
      testCh_++;
      testStep_ = 0;
      Serial.print(F("[SERVO] 切换到下一路方向标定: "));
      Serial.println(kNames[testCh_]);
      setAngle(testCh_, SERVO_CENTER_DEG);
    } else {
      testing_ = false;
      Serial.println(F("[SERVO] 方向标定完成"));
    }
  }
}

bool ServoController::isAttached(uint8_t idx) const {
  return (idx < SERVO_COUNT) ? attached_[idx] : false;
}

uint8_t ServoController::currentAngle(uint8_t idx) const {
  return (idx < SERVO_COUNT) ? angles_[idx] : 0;
}

void ServoController::printStatus() const {
  Serial.println(F("  Servos (Timer5, 竖直收回->外扩 135~140 deg):"));
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    Serial.print(F("    "));
    Serial.print(kNames[i]);
    Serial.print(F(" [in=")); Serial.print(kIdleDeg[i]);
    Serial.print(F(" out=")); Serial.print(kStrikeDeg[i]);
    Serial.print(F("] : "));
    if (attached_[i]) {
      Serial.print(angles_[i]);
      Serial.println(F(" deg (ON)"));
    } else {
      Serial.println(F("OFF"));
    }
  }
}
