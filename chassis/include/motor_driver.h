#pragma once

#include <Arduino.h>
#include "config.h"

class EncoderArray;

enum WheelId : uint8_t {
  WHEEL_FL = 0,
  WHEEL_RL = 1,
  WHEEL_FR = 2,
  WHEEL_RR = 3,
  WHEEL_COUNT = 4
};

struct MotorChannelConfig {
  uint8_t enPin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  bool reversed;
};

struct MotorParams {
  float scale[WHEEL_COUNT] = {MOTOR_FL_SCALE, MOTOR_RL_SCALE, MOTOR_FR_SCALE, MOTOR_RR_SCALE};
  bool velPidEnabled       = DEFAULT_VEL_PID_ENABLED;
  float maxSpeedMmS        = DEFAULT_WHEEL_MAX_MMS;
  float velKp              = DEFAULT_VEL_KP;
  float velKi              = DEFAULT_VEL_KI;
};

class MotorDriver {
 public:
  void begin();

  // 设置单个轮子目标 PWM（开环直接输出；开启速度闭环时按比例映射为目标轮速 mm/s）
  void setWheel(WheelId wheel, int16_t pwm);

  // 设置左右两侧 PWM（左前+左后同速，右前+右后同速）
  void setLeftRight(int16_t leftPwm, int16_t rightPwm);

  // 差速控制：forwardPwm > 0 前进，turnPwm > 0 右转，turnPwm < 0 左转
  void setTank(int16_t forwardPwm, int16_t turnPwm);

  // 滑行停止（EN=0, IN1=0, IN2=0）
  void stopAll();

  // 短接制动（IN1=1, IN2=1, EN=HIGH），注意仅短时使用
  void brakeAll();

  // 开启速度闭环时在主循环中调用（与 EncoderArray 同步 20 Hz 刷新），含编码器断线/反相防飞车保护
  void updateVelocityLoop(const EncoderArray& encoders, uint32_t nowMs);

  void resetVelocityPid();

  MotorParams& params() { return params_; }
  const MotorParams& params() const { return params_; }

  int16_t getCommandPwm(WheelId wheel) const;
  int16_t getAppliedPwm(WheelId wheel) const;
  static const char* wheelName(WheelId wheel);

 private:
  void applyHardwarePwm(WheelId wheel, int16_t hwPwm);

  static const MotorChannelConfig kChannels[WHEEL_COUNT];
  MotorParams params_;
  int16_t cmdPwm_[WHEEL_COUNT] = {0, 0, 0, 0};
  int16_t appliedPwm_[WHEEL_COUNT] = {0, 0, 0, 0};
  int8_t lastDir_[WHEEL_COUNT] = {0, 0, 0, 0};

  float velInteg_[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint32_t faultSinceMs_[WHEEL_COUNT] = {0, 0, 0, 0};
  uint32_t lastVelUpdateMs_ = 0;
};
