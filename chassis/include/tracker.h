#pragma once

#include <Arduino.h>
#include "config.h"
#include "imu_serial.h"
#include "motor_driver.h"
#include "ultrasonic_array.h"

enum TrackerMode : uint8_t {
  MODE_IDLE = 0,
  MODE_TRACK_FWD,
  MODE_TRACK_REV,
  MODE_STOPPED_END,
  MODE_MANUAL_TEST
};

enum WallMode : uint8_t {
  WALL_NONE = 0,
  WALL_BOTH,
  WALL_LEFT_ONLY,
  WALL_RIGHT_ONLY
};

struct TrackerParams {
  int16_t basePwm         = DEFAULT_BASE_PWM;
  int16_t slowPwm         = DEFAULT_SLOW_PWM;
  uint16_t frontSlowMm    = DEFAULT_FRONT_SLOW_MM;
  uint16_t frontStopMm    = DEFAULT_FRONT_STOP_MM;
  uint16_t rearStopMm     = DEFAULT_REAR_STOP_MM;
  uint16_t sideWallMaxMm  = DEFAULT_SIDE_WALL_MAX_MM;
  uint16_t singleWallMm   = DEFAULT_SINGLE_WALL_MM;
  int16_t errDeadbandMm   = DEFAULT_ERR_DEADBAND_MM;
  float kp                = DEFAULT_KP_CENTER;
  float kd                = DEFAULT_KD_CENTER;
  float kpYawHold         = DEFAULT_KP_YAW_HOLD;
  int16_t maxSteerPwm     = DEFAULT_MAX_STEER_PWM;
};

struct TrackerStatus {
  TrackerMode mode = MODE_IDLE;
  WallMode wallMode = WALL_NONE;
  int16_t errMm = 0;
  float yawErrDeg = 0.0f;
  bool headingLocked = false;
  int16_t cmdForward = 0;
  int16_t cmdTurn = 0;
};

class CorridorTracker {
 public:
  void begin(MotorDriver* motors, UltrasonicArray* sensors, ImuSerial* imu = nullptr);

  void stop();
  void startForward(uint32_t nowMs);
  void startForwardWithHeading(float targetYawDeg);
  void startReverse(uint32_t nowMs);
  void startReverseWithHeading(float targetYawDeg);
  void startManualTest(uint32_t nowMs, uint32_t durationMs);

  void update(uint32_t nowMs);

  bool hasReachedEnd() const { return status_.mode == MODE_STOPPED_END; }

  TrackerParams& params() { return params_; }
  const TrackerParams& params() const { return params_; }
  const TrackerStatus& status() const { return status_; }

  static const char* modeName(TrackerMode mode);
  static const char* wallModeName(WallMode mode);

 private:
  void resetController();
  void runTracking(uint32_t nowMs, bool forward);

  MotorDriver* motors_ = nullptr;
  UltrasonicArray* sensors_ = nullptr;
  ImuSerial* imu_ = nullptr;
  TrackerParams params_;
  TrackerStatus status_;

  bool useHeadingHold_ = false;
  float targetYawDeg_ = 0.0f;
  uint32_t trackStartMs_ = 0;

  WallMode prevWallMode_ = WALL_NONE;
  bool prevErrValid_ = false;
  int16_t prevErrMm_ = 0;
  uint32_t lastSideUpdateMs_ = 0;
  float filteredDErr_ = 0.0f;
  uint8_t endStopStreak_ = 0;
  uint32_t lastLeadUpdateMs_ = 0;
  uint32_t testDeadlineMs_ = 0;
};
