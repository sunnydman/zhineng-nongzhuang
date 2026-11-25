#include "servo_controller.h"

const uint8_t ServoController::kPins[SERVO_COUNT] = {
    PIN_SERVO_0,
    PIN_SERVO_1,
    PIN_SERVO_2,
};

void ServoController::begin() {
  // 上电保持不挂载 PWM，避免未知初始位置时舵机猛甩撞击机械限位
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    attached_[i] = false;
    angles_[i] = SERVO_CENTER_DEG;
  }
  testing_ = false;
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

uint8_t ServoController::pinOf(uint8_t idx) {
  return (idx < SERVO_COUNT) ? kPins[idx] : 0;
}

void ServoController::setAngle(uint8_t idx, uint8_t deg) {
  if (idx >= SERVO_COUNT) {
    return;
  }
  if (deg > 180) {
    deg = 180;
  }
  if (!attached_[idx]) {
    servos_[idx].write(deg);
    servos_[idx].attach(kPins[idx], SERVO_MIN_US, SERVO_MAX_US);
    attached_[idx] = true;
  } else {
    servos_[idx].write(deg);
  }
  angles_[idx] = deg;
}

void ServoController::setAll(uint8_t deg) {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    setAngle(i, deg);
  }
}

void ServoController::detachAll() {
  testing_ = false;
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
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

  Serial.print(F("[SERVO] 开始小幅连通性测试 (90 -> "));
  Serial.print(SERVO_CENTER_DEG - SERVO_TEST_OFFSET);
  Serial.print(F(" -> "));
  Serial.print(SERVO_CENTER_DEG + SERVO_TEST_OFFSET);
  Serial.print(F(" -> 90 deg): D"));
  Serial.println(kPins[testCh_]);
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

  Serial.print(F("[SERVO] 启动持续往复摆动测试 (20 <-> 160 deg, 周期 1.2s): D"));
  Serial.println(kPins[testCh_]);
  setAngle(testCh_, 20);
}

void ServoController::stopTest() {
  testing_ = false;
  loopMode_ = false;
}

void ServoController::update(uint32_t nowMs) {
  if (!testing_) {
    return;
  }

  // 脱机循环测试模式：20° <-> 160° 每 1200ms 自动来回甩动，方便拔掉 USB 用万用表调 6V 供电
  if (loopMode_) {
    if ((nowMs - testStepStartMs_) < 1200UL) {
      return;
    }
    testStepStartMs_ = nowMs;
    testStep_ = (testStep_ == 0) ? 1 : 0;
    uint8_t deg = (testStep_ == 0) ? 20 : 160;
    setAngle(testCh_, deg);
    return;
  }

  if ((nowMs - testStepStartMs_) < SERVO_TEST_STEP_MS) {
    return;
  }
  testStepStartMs_ = nowMs;
  testStep_++;

  if (testStep_ == 1) {
    uint8_t lo = SERVO_CENTER_DEG - SERVO_TEST_OFFSET;
    Serial.print(F("[SERVO] D")); Serial.print(kPins[testCh_]);
    Serial.print(F(" -> ")); Serial.print(lo); Serial.println(F(" deg"));
    setAngle(testCh_, lo);
  } else if (testStep_ == 2) {
    uint8_t hi = SERVO_CENTER_DEG + SERVO_TEST_OFFSET;
    Serial.print(F("[SERVO] D")); Serial.print(kPins[testCh_]);
    Serial.print(F(" -> ")); Serial.print(hi); Serial.println(F(" deg"));
    setAngle(testCh_, hi);
  } else if (testStep_ == 3) {
    Serial.print(F("[SERVO] D")); Serial.print(kPins[testCh_]);
    Serial.print(F(" -> ")); Serial.print(SERVO_CENTER_DEG); Serial.println(F(" deg (归中)"));
    setAngle(testCh_, SERVO_CENTER_DEG);
  } else {
    // 当前通道测完，若测全部则切下一路
    if (testAll_ && (testCh_ + 1) < SERVO_COUNT) {
      testCh_++;
      testStep_ = 0;
      Serial.print(F("[SERVO] 切换到下一路测试: D"));
      Serial.println(kPins[testCh_]);
      setAngle(testCh_, SERVO_CENTER_DEG);
    } else {
      testing_ = false;
      Serial.println(F("[SERVO] 舵机测试完成（发 SV OFF 可断力释放，或发 SV <32|33|34> <角度> 精调）"));
    }
  }
}

bool ServoController::isAttached(uint8_t idx) const {
  return (idx < SERVO_COUNT) ? attached_[idx] : false;
}

uint8_t ServoController::currentAngle(uint8_t idx) const {
  return (idx < SERVO_COUNT) ? angles_[idx] : 0;
}
