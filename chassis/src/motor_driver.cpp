#include "motor_driver.h"

#include "encoder_array.h"

const MotorChannelConfig MotorDriver::kChannels[WHEEL_COUNT] = {
    {PIN_MOTOR_FL_EN, PIN_MOTOR_FL_IN1, PIN_MOTOR_FL_IN2, MOTOR_FL_REVERSED},
    {PIN_MOTOR_RL_EN, PIN_MOTOR_RL_IN1, PIN_MOTOR_RL_IN2, MOTOR_RL_REVERSED},
    {PIN_MOTOR_FR_EN, PIN_MOTOR_FR_IN1, PIN_MOTOR_FR_IN2, MOTOR_FR_REVERSED},
    {PIN_MOTOR_RR_EN, PIN_MOTOR_RR_IN1, PIN_MOTOR_RR_IN2, MOTOR_RR_REVERSED},
};

void MotorDriver::begin() {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const auto& ch = kChannels[i];
    // 先拉低使能脚，再设为输出，避免上电/复位瞬间电机抖动
    digitalWrite(ch.enPin, LOW);
    digitalWrite(ch.in1Pin, LOW);
    digitalWrite(ch.in2Pin, LOW);
    pinMode(ch.enPin, OUTPUT);
    pinMode(ch.in1Pin, OUTPUT);
    pinMode(ch.in2Pin, OUTPUT);
    analogWrite(ch.enPin, 0);
    cmdPwm_[i] = 0;
    appliedPwm_[i] = 0;
    lastDir_[i] = 0;
  }
  resetVelocityPid();
}

void MotorDriver::resetVelocityPid() {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    velInteg_[i] = 0.0f;
    faultSinceMs_[i] = 0;
  }
  lastVelUpdateMs_ = 0;
}

void MotorDriver::applyHardwarePwm(WheelId wheel, int16_t hwPwm) {
  const auto& ch = kChannels[wheel];
  hwPwm = constrain(hwPwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  appliedPwm_[wheel] = hwPwm;

  int16_t phys = ch.reversed ? -hwPwm : hwPwm;

  int8_t dir = 0;
  uint8_t duty = 0;
  if (phys > 0) {
    dir = 1;
    duty = static_cast<uint8_t>(phys);
  } else if (phys < 0) {
    dir = -1;
    duty = static_cast<uint8_t>(-phys);
  }

  if (duty > 0 && duty < MOTOR_PWM_MIN_EFFECTIVE) {
    duty = static_cast<uint8_t>(MOTOR_PWM_MIN_EFFECTIVE);
  }

  // 遵循 L298N 换向保护：改变方向前先将 EN 降为 0 并等待短死区
  if (dir != lastDir_[wheel]) {
    analogWrite(ch.enPin, 0);
    digitalWrite(ch.enPin, LOW);
    if (lastDir_[wheel] != 0 && dir != 0) {
      delayMicroseconds(MOTOR_DIR_DEADTIME_US);
    }

    if (dir > 0) {
      digitalWrite(ch.in1Pin, HIGH);
      digitalWrite(ch.in2Pin, LOW);
    } else if (dir < 0) {
      digitalWrite(ch.in1Pin, LOW);
      digitalWrite(ch.in2Pin, HIGH);
    } else {
      digitalWrite(ch.in1Pin, LOW);
      digitalWrite(ch.in2Pin, LOW);
    }
    lastDir_[wheel] = dir;
  }

  if (duty == 0) {
    analogWrite(ch.enPin, 0);
    digitalWrite(ch.enPin, LOW);
  } else {
    analogWrite(ch.enPin, duty);
  }
}

void MotorDriver::setWheel(WheelId wheel, int16_t pwm) {
  if (wheel >= WHEEL_COUNT) {
    return;
  }

  pwm = constrain(pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  cmdPwm_[wheel] = pwm;

  if (pwm == 0) {
    velInteg_[wheel] = 0.0f;
    faultSinceMs_[wheel] = 0;
    applyHardwarePwm(wheel, 0);
    return;
  }

  if (!params_.velPidEnabled) {
    int16_t scaled = static_cast<int16_t>(static_cast<float>(pwm) * params_.scale[wheel]);
    applyHardwarePwm(wheel, scaled);
  } else if (appliedPwm_[wheel] == 0) {
    // 闭环刚起步时先给一次前馈输出，不等下个 50 ms 采样周期
    int16_t ff = static_cast<int16_t>(static_cast<float>(pwm) * params_.scale[wheel]);
    applyHardwarePwm(wheel, ff);
  }
}

void MotorDriver::updateVelocityLoop(const EncoderArray& encoders, uint32_t nowMs) {
  if (!params_.velPidEnabled) {
    return;
  }

  if (lastVelUpdateMs_ != 0 && (nowMs - lastVelUpdateMs_) < ENCODER_SAMPLE_MS) {
    return;
  }
  float dtSec = (lastVelUpdateMs_ == 0)
                    ? (static_cast<float>(ENCODER_SAMPLE_MS) * 0.001f)
                    : (static_cast<float>(nowMs - lastVelUpdateMs_) * 0.001f);
  lastVelUpdateMs_ = nowMs;
  dtSec = constrain(dtSec, 0.02f, 0.25f);

  const float speedPerPwm = params_.maxSpeedMmS / static_cast<float>(MOTOR_PWM_MAX);

  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    WheelId w = static_cast<WheelId>(i);
    int16_t cmd = cmdPwm_[i];
    if (cmd == 0) {
      velInteg_[i] = 0.0f;
      faultSinceMs_[i] = 0;
      applyHardwarePwm(w, 0);
      continue;
    }

    float targetMmS = static_cast<float>(cmd) * speedPerPwm;
    float measMmS = encoders.speedMmPerSec(w);

    // 防飞车安全检测：给足驱动 PWM 后，若编码器无响应或与目标反号超过 VEL_FAULT_MS，立刻降回开环
    bool badEncoder = false;
    if (abs(cmd) >= 60) {
      if (cmd > 0 && measMmS < 5.0f) {
        badEncoder = true;
      } else if (cmd < 0 && measMmS > -5.0f) {
        badEncoder = true;
      }
    }

    if (badEncoder) {
      if (faultSinceMs_[i] == 0) {
        faultSinceMs_[i] = nowMs;
      } else if ((nowMs - faultSinceMs_[i]) >= VEL_FAULT_MS) {
        params_.velPidEnabled = false;
        resetVelocityPid();
        Serial.print(F("[SAFE] Encoder fault/reversed on "));
        Serial.print(wheelName(w));
        Serial.println(F(" -> Auto-disabled VPID, falling back to open-loop PWM!"));
        for (uint8_t k = 0; k < WHEEL_COUNT; ++k) {
          int16_t fallback = static_cast<int16_t>(static_cast<float>(cmdPwm_[k]) * params_.scale[k]);
          applyHardwarePwm(static_cast<WheelId>(k), fallback);
        }
        return;
      }
    } else {
      faultSinceMs_[i] = 0;
    }

    float err = targetMmS - measMmS;
    velInteg_[i] += params_.velKi * err * dtSec;
    velInteg_[i] = constrain(velInteg_[i], -VEL_INTEGRAL_LIMIT, VEL_INTEGRAL_LIMIT);

    // 将前馈与 PI 统一按单轮 scale 缩放，保证四轮具有一致的环路阻尼，消除速度闭环喘振
    float out = (static_cast<float>(cmd) + params_.velKp * err + velInteg_[i]) * params_.scale[i];
    int16_t hwPwm = static_cast<int16_t>(out);

    // 闭环时不允许反向反推（避免目标前进时刹车过度出现正反转拉扯）
    if (cmd > 0 && hwPwm < 0) hwPwm = 0;
    if (cmd < 0 && hwPwm > 0) hwPwm = 0;

    applyHardwarePwm(w, hwPwm);
  }
}

void MotorDriver::setLeftRight(int16_t leftPwm, int16_t rightPwm) {
  setWheel(WHEEL_FL, leftPwm);
  setWheel(WHEEL_RL, leftPwm);
  setWheel(WHEEL_FR, rightPwm);
  setWheel(WHEEL_RR, rightPwm);
}

void MotorDriver::setTank(int16_t forwardPwm, int16_t turnPwm) {
  int16_t leftPwm = constrain(forwardPwm + turnPwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  int16_t rightPwm = constrain(forwardPwm - turnPwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  setLeftRight(leftPwm, rightPwm);
}

void MotorDriver::stopAll() {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const auto& ch = kChannels[i];
    analogWrite(ch.enPin, 0);
    digitalWrite(ch.enPin, LOW);
    digitalWrite(ch.in1Pin, LOW);
    digitalWrite(ch.in2Pin, LOW);
    cmdPwm_[i] = 0;
    appliedPwm_[i] = 0;
    lastDir_[i] = 0;
  }
  resetVelocityPid();
}

void MotorDriver::brakeAll() {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const auto& ch = kChannels[i];
    digitalWrite(ch.in1Pin, HIGH);
    digitalWrite(ch.in2Pin, HIGH);
    analogWrite(ch.enPin, MOTOR_PWM_MAX);
    cmdPwm_[i] = 0;
    appliedPwm_[i] = 0;
    lastDir_[i] = 0;
  }
  resetVelocityPid();
}

int16_t MotorDriver::getCommandPwm(WheelId wheel) const {
  if (wheel >= WHEEL_COUNT) {
    return 0;
  }
  return cmdPwm_[wheel];
}

int16_t MotorDriver::getAppliedPwm(WheelId wheel) const {
  if (wheel >= WHEEL_COUNT) {
    return 0;
  }
  return appliedPwm_[wheel];
}

const char* MotorDriver::wheelName(WheelId wheel) {
  switch (wheel) {
    case WHEEL_FL: return "FL";
    case WHEEL_RL: return "RL";
    case WHEEL_FR: return "FR";
    case WHEEL_RR: return "RR";
    default:       return "??";
  }
}
