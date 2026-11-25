// ============================================================================
// 主程序入口（Arduino Mega 2560 + Pro Max V3.3 扩展板）
// 职责：
//   1. 按固定优先级调度各子系统：串口急停/指令 -> 按键检测 -> IMU解析 -> 超声轮询
//      -> 编码器测速 -> 垄沟循迹 -> 全程状态机 -> 四轮速度闭环 PID -> 状态灯/遥测
//   2. 提供完整的串口在线调试命令（单轮测试、定距直行、90度转弯、单次U弯、全程跑图、
//      在线调参 SET、掉电保存 SAVE）
// ============================================================================
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "encoder_array.h"
#include "imu_serial.h"
#include "mission.h"
#include "motor_driver.h"
#include "servo_controller.h"
#include "storage.h"
#include "tracker.h"
#include "ultrasonic_array.h"

namespace {

// ---------------------- 全局核心子系统实例 ----------------------
MotorDriver gMotors;         // 四路直流电机驱动 + 四轮独立速度 PI 闭环
UltrasonicArray gUltrasonics;// 四路 HC-SR04 超声波轮询 + 3 点中值滤波
EncoderArray gEncoders;      // 四路霍尔正交编码器（PCINT2 4 倍频）
ImuSerial gImu;              // Serial1 串口六轴 IMU（0x7E 0x23 协议）
CorridorTracker gTracker;    // 垄沟居中循迹 + IMU 锁直 + 端头防撞控制器
MissionController gMission;  // 全程闭环与单步测试任务状态机
ServoController gServos;     // 三路舵机控制器（D32 / D33 / D34，占用 Timer5）

// 串口实时遥测开关与节拍
bool gStreamTelemetry = false;
uint32_t gLastTelemetryMs = 0;

// 发车倒计时状态（用于实体按键或上电自动发车，期间可随时急停取消）
bool gArmedCountdown = false;
uint32_t gLaunchAtMs = 0;
uint8_t gLaunchMode = 2;  // 1=L形测试(直行->转弯->直行)，2=全程比赛闭环
uint8_t gLaunchLanes = AUTO_START_LANES;

// 实体启动/急停按键消抖状态
bool gBtnLastRaw = HIGH;
bool gBtnStable = HIGH;
uint32_t gBtnLastChangeMs = 0;

// 串口命令行缓冲
constexpr size_t kCmdBufSize = 64;
char gCmdBuf[kCmdBufSize];
size_t gCmdLen = 0;

// 立即取消一切倒计时、任务、循迹与舵机测试，并停机
void cancelAll() {
  gArmedCountdown = false;
  gMission.abort();
  gServos.stopTest();
}

// 预约或立即启动全程比赛任务（delayMs=0 表示立即发车）
void armFullMatch(uint8_t lanes, uint16_t delayMs, uint32_t nowMs) {
  cancelAll();
  gLaunchMode = 2;
  gLaunchLanes = lanes;
  if (delayMs == 0) {
    gMission.startFullMatch(gLaunchLanes, nowMs);
  } else {
    gArmedCountdown = true;
    gLaunchAtMs = nowMs + delayMs;
    Serial.print(F("[ARM] Full match ("));
    Serial.print(gLaunchLanes);
    Serial.print(F(" lanes) starting in "));
    Serial.print(delayMs);
    Serial.println(F(" ms... (press button or send X/Space to cancel)"));
  }
}

// 预约启动上电自动 L 形测试（直行 -> 90°转弯 -> 直行）
void armAutoLTest(uint16_t delayMs, uint32_t nowMs) {
  cancelAll();
  gLaunchMode = 1;
  if (delayMs == 0) {
    gMission.startLTest(
        AUTO_LTEST_LEG1_MM, AUTO_LTEST_TURN_LEFT, AUTO_LTEST_LEG2_MM, AUTO_LTEST_ARC_SIGN, nowMs);
  } else {
    gArmedCountdown = true;
    gLaunchAtMs = nowMs + delayMs;
    Serial.print(F("[ARM] Auto L-Test ("));
    Serial.print(AUTO_LTEST_LEG1_MM);
    Serial.print(AUTO_LTEST_TURN_LEFT ? F("mm -> L90 -> ") : F("mm -> R90 -> "));
    Serial.print(AUTO_LTEST_LEG2_MM);
    Serial.print(F("mm) starting in "));
    Serial.print(delayMs);
    Serial.println(F(" ms... (press button or send X/Space to cancel)"));
  }
}

void printPinLabel(uint8_t pin) {
  if (pin >= A0 && pin <= A15) {
    Serial.print('A');
    Serial.print(pin - A0);
  } else {
    Serial.print('D');
    Serial.print(pin);
  }
}

void printHelp() {
  Serial.println(F("\n=== 4WD + 4-Ultrasonic + 6-Axis IMU Controller (Mega 2560 Pro Max V3.3) ==="));
  Serial.println(F("  X / Space            : Emergency STOP (abort mission & stop all motors)"));
  Serial.println(F("  G / B                : Start FORWARD / REVERSE single-lane tracking"));
  Serial.println(F("  D <mm>               : Drive straight exact encoder distance (e.g. D 500 or D -300)"));
  Serial.println(F("  LT [d1] [L|R] [d2] [A]: L-shape test: Straight -> Turn 90 -> Straight (default: 800 R 800)"));
  Serial.println(F("  TL / TR              : Test in-place 90 deg LEFT / RIGHT spin"));
  Serial.println(F("  AL / AR              : Test forward-biased ARC 90 deg LEFT / RIGHT corner turn"));
  Serial.println(F("  UL / UR              : Test full U-turn lane change (LEFT / RIGHT)"));
  Serial.println(F("  M [lanes] [L|R]      : Start snake mission (default: M 7 L)"));
  Serial.println(F("  F [lanes]            : Start FULL MATCH (Exit -> Ch7..Ch1 -> Reverse -> Park Home)"));
  Serial.println(F("  U                    : Toggle live telemetry (US + IMU + State)"));
  Serial.println(F("  Z                    : Zero IMU yaw reference (set current heading = 0 deg)"));
  Serial.println(F("  I                    : Sniff raw Serial1 HEX bytes for 1 s (verify IMU protocol)"));
  Serial.println(F("  CAL / VER            : Send IMU gyro calibrate (0x70) / version query (0x80)"));
  Serial.println(F("  RATE <10..100>       : Set IMU output frequency Hz (0x60)"));
  Serial.println(F("  BAUD <rate>          : Switch Serial1 baud rate online (e.g. 115200 or 9600)"));
  Serial.println(F("  W <target> <pwm> [ms]: Bench test motor (target: fl|rl|fr|rr|l|r|all)"));
  Serial.println(F("  SV [32|33|34|all] <deg|test|off>: Test/move servos on D32/D33/D34 (e.g. SV, SV 32 90, SV OFF)"));
  Serial.println(F("  SCALE <wheel> <val>  : Trim single wheel PWM scale (fl|rl|fr|rr, 0.50..1.30)"));
  Serial.println(F("  E / EC               : Print 4-wheel encoder counts/speed / Clear encoder counts"));
  Serial.println(F("  P                    : Print pinout, IMU status & parameters"));
  Serial.println(F("  SET <key> <val>      : Tune params online:"));
  Serial.println(F("                         kp kd ykp spd slow stop rstop wall side steer"));
  Serial.println(F("                         tpwm tmin tkp arc tms cpwm cout cridge copen cmm cmax hdist"));
  Serial.println(F("                         vpid(0|1) vkp vki vmax"));
  Serial.println(F("  SAVE / LOAD / RESETCFG: Save/reload/reset tuned params in EEPROM"));
  Serial.println(F("  H / ?                : Show this help\n"));
}

void printConfigSummary() {
  const auto& mp0 = gMotors.params();
  const auto& p = gTracker.params();
  const auto& mp = gMission.params();
  uint32_t nowMs = millis();

  Serial.println(F("\n--- Pinout (PCB 25.11.19 - V3.3) ---"));
  Serial.print(F("  FL: EN=D")); Serial.print(PIN_MOTOR_FL_EN);
  Serial.print(F(" IN1=D")); Serial.print(PIN_MOTOR_FL_IN1);
  Serial.print(F(" IN2=D")); Serial.print(PIN_MOTOR_FL_IN2);
  Serial.print(F(" rev=")); Serial.print(MOTOR_FL_REVERSED ? F("Y") : F("N"));
  Serial.print(F(" scale=")); Serial.println(mp0.scale[WHEEL_FL], 2);

  Serial.print(F("  RL: EN=D")); Serial.print(PIN_MOTOR_RL_EN);
  Serial.print(F(" IN1=D")); Serial.print(PIN_MOTOR_RL_IN1);
  Serial.print(F(" IN2=D")); Serial.print(PIN_MOTOR_RL_IN2);
  Serial.print(F(" rev=")); Serial.print(MOTOR_RL_REVERSED ? F("Y") : F("N"));
  Serial.print(F(" scale=")); Serial.println(mp0.scale[WHEEL_RL], 2);

  Serial.print(F("  FR: EN=D")); Serial.print(PIN_MOTOR_FR_EN);
  Serial.print(F(" IN1=D")); Serial.print(PIN_MOTOR_FR_IN1);
  Serial.print(F(" IN2=D")); Serial.print(PIN_MOTOR_FR_IN2);
  Serial.print(F(" rev=")); Serial.print(MOTOR_FR_REVERSED ? F("Y") : F("N"));
  Serial.print(F(" scale=")); Serial.println(mp0.scale[WHEEL_FR], 2);

  Serial.print(F("  RR: EN=D")); Serial.print(PIN_MOTOR_RR_EN);
  Serial.print(F(" IN1=D")); Serial.print(PIN_MOTOR_RR_IN1);
  Serial.print(F(" IN2=D")); Serial.print(PIN_MOTOR_RR_IN2);
  Serial.print(F(" rev=")); Serial.print(MOTOR_RR_REVERSED ? F("Y") : F("N"));
  Serial.print(F(" scale=")); Serial.println(mp0.scale[WHEEL_RR], 2);

  Serial.print(F("  VelPID: ")); Serial.print(mp0.velPidEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" vkp=")); Serial.print(mp0.velKp, 2);
  Serial.print(F(" vki=")); Serial.print(mp0.velKi, 2);
  Serial.print(F(" vmax=")); Serial.print(mp0.maxSpeedMmS, 0);
  Serial.println(F(" mm/s"));

  Serial.print(F("  Ultrasonic (Trig/Echo): F="));
  printPinLabel(PIN_US_FRONT_TRIG); Serial.print('/'); printPinLabel(PIN_US_FRONT_ECHO);
  Serial.print(F(" B="));
  printPinLabel(PIN_US_REAR_TRIG); Serial.print('/'); printPinLabel(PIN_US_REAR_ECHO);
  Serial.print(F(" L="));
  printPinLabel(PIN_US_LEFT_TRIG); Serial.print('/'); printPinLabel(PIN_US_LEFT_ECHO);
  Serial.print(F(" R="));
  printPinLabel(PIN_US_RIGHT_TRIG); Serial.print('/'); printPinLabel(PIN_US_RIGHT_ECHO);
  Serial.println();

  Serial.print(F("  IMU (Serial1 R19/T18, ")); Serial.print(ImuSerial::protocolName());
  Serial.print(F("): baud=")); Serial.print(gImu.baud());
  Serial.print(F(" online=")); Serial.print(gImu.isOnline(nowMs) ? F("YES") : F("NO"));
  Serial.print(F(" pkts=")); Serial.print(gImu.eulerPacketCount());
  Serial.print(F(" yaw=")); Serial.print(gImu.yawDeg(), 1);
  Serial.print(F(" deg gz=")); Serial.print(gImu.gyroZDegPerSec(), 1);
  Serial.println(F(" deg/s"));

  Serial.println(F("  Encoders (PCINT2 A8-A15): FL=A12/A13, RL=A8/A9, FR=A14/A15, RR=A10/A11"));
  Serial.print(F("  Servos (Timer5): D")); Serial.print(PIN_SERVO_0);
  Serial.print(F(", D")); Serial.print(PIN_SERVO_1);
  Serial.print(F(", D")); Serial.println(PIN_SERVO_2);

  Serial.println(F("--- Tracker Params ---"));
  Serial.print(F("  kp=")); Serial.print(p.kp, 3);
  Serial.print(F(" kd=")); Serial.print(p.kd, 3);
  Serial.print(F(" ykp=")); Serial.print(p.kpYawHold, 2);
  Serial.print(F(" spd=")); Serial.print(p.basePwm);
  Serial.print(F(" slow=")); Serial.print(p.slowPwm);
  Serial.print(F(" steer=")); Serial.println(p.maxSteerPwm);

  Serial.print(F("  stop=")); Serial.print(p.frontStopMm);
  Serial.print(F("mm rstop=")); Serial.print(p.rearStopMm);
  Serial.print(F("mm wall=")); Serial.print(p.sideWallMaxMm);
  Serial.print(F("mm side=")); Serial.print(p.singleWallMm);
  Serial.println(F("mm"));

  Serial.println(F("--- Turn & Lane-Change Params (L=320 W=310 recess=40mm) ---"));
  Serial.print(F("  tpwm=")); Serial.print(mp.turnMaxPwm);
  Serial.print(F(" tmin=")); Serial.print(mp.turnMinPwm);
  Serial.print(F(" tkp=")); Serial.print(mp.turnKp, 2);
  Serial.print(F(" arc=")); Serial.print(mp.turnArcRatio, 2);
  Serial.print(F(" tms=")); Serial.println(mp.turnFallback90Ms);

  Serial.print(F("  cpwm=")); Serial.print(mp.crossPwm);
  Serial.print(F(" cout=")); Serial.print(mp.crossOuterWallMm);
  Serial.print(F("mm cridge=")); Serial.print(mp.crossRidgeMaxMm);
  Serial.print(F("mm copen=")); Serial.print(mp.crossOpenMinMm);
  Serial.print(F("mm cmm=")); Serial.print(mp.crossCenterMm);
  Serial.print(F("mm cmax=")); Serial.print(mp.crossMaxDistMm);
  Serial.print(F("mm hdist=")); Serial.print(mp.homeDistMm);
  Serial.println(F("mm\n"));
}

void printDistanceField(UltrasonicId id) {
  const auto& r = gUltrasonics.get(id);
  Serial.print(UltrasonicArray::sensorName(id));
  Serial.print('=');
  if (r.valid) {
    Serial.print(r.filteredMm);
  } else {
    Serial.print(F("---"));
  }
}

void printTelemetry(uint32_t nowMs) {
  const auto& st = gTracker.status();
  Serial.print('[');
  if (gMission.isActive()) {
    Serial.print(F("M:"));
    Serial.print(MissionController::stateName(gMission.state()));
    if (gMission.state() == MS_CROSS_RIDGE) {
      Serial.print(':');
      Serial.print(MissionController::crossPhaseName(gMission.crossPhase()));
    } else if (gMission.totalLanes() > 0) {
      Serial.print(' ');
      Serial.print(gMission.currentLane());
      Serial.print('/');
      Serial.print(gMission.totalLanes());
    }
  } else {
    Serial.print(CorridorTracker::modeName(st.mode));
    Serial.print('|');
    Serial.print(CorridorTracker::wallModeName(st.wallMode));
  }
  Serial.print(F("] "));

  printDistanceField(US_FRONT); Serial.print(' ');
  printDistanceField(US_REAR);  Serial.print(' ');
  printDistanceField(US_LEFT);  Serial.print(' ');
  printDistanceField(US_RIGHT);

  Serial.print(F(" | Y="));
  if (gImu.isOnline(nowMs)) {
    Serial.print(gImu.yawDeg(), 1);
  } else {
    Serial.print(F("OFF"));
  }

  Serial.print(F(" | err=")); Serial.print(st.errMm);
  Serial.print(F(" | L=")); Serial.print(gMotors.getCommandPwm(WHEEL_FL));
  Serial.print(F(" R=")); Serial.print(gMotors.getCommandPwm(WHEEL_FR));
  if (gEncoders.anyActivity()) {
    Serial.print(F(" | V=")); Serial.print(gEncoders.avgSpeedMmPerSec(), 0);
    Serial.print(F("mm/s D=")); Serial.print(gEncoders.avgDistanceMm(), 0);
    Serial.print(F("mm"));
  }
  Serial.println();
}

void printEncoderSummary() {
  Serial.println(F("\n--- 4-Wheel Encoders (AB x4, PCINT2 A8-A15) ---"));
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    WheelId w = static_cast<WheelId>(i);
    Serial.print(F("  "));
    Serial.print(MotorDriver::wheelName(w));
    Serial.print(F(": cnt=")); Serial.print(gEncoders.counts(w));
    Serial.print(F("  dist=")); Serial.print(gEncoders.distanceMm(w), 1);
    Serial.print(F(" mm  spd=")); Serial.print(gEncoders.speedMmPerSec(w), 1);
    Serial.println(F(" mm/s"));
  }
  Serial.print(F("  AVG: dist=")); Serial.print(gEncoders.avgDistanceMm(), 1);
  Serial.print(F(" mm  spd=")); Serial.print(gEncoders.avgSpeedMmPerSec(), 1);
  Serial.println(F(" mm/s"));
  Serial.print(F("  RAW PINK=0x")); Serial.print(gEncoders.rawPink(), HEX);
  Serial.print(F(" edges[A8..A15]="));
  for (uint8_t b = 0; b < 8; ++b) {
    Serial.print(gEncoders.rawPinEdges(b));
    if (b < 7) Serial.print(',');
  }
  Serial.println(F("\n"));
}

void toLowerStr(char* s) {
  for (; *s != '\0'; ++s) {
    *s = static_cast<char>(tolower(static_cast<unsigned char>(*s)));
  }
}

void handleWheelTest(char* argTarget, char* argPwm, char* argDur) {
  if (argTarget == nullptr || argPwm == nullptr) {
    Serial.println(F("[ERR] Usage: W <fl|rl|fr|rr|l|r|all> <pwm> [ms]"));
    return;
  }

  gMission.abort();
  toLowerStr(argTarget);
  int16_t pwm = static_cast<int16_t>(atoi(argPwm));
  uint32_t durMs = (argDur != nullptr) ? static_cast<uint32_t>(atol(argDur)) : TEST_DEFAULT_DUR_MS;
  if (durMs == 0 || durMs > 10000UL) {
    durMs = TEST_DEFAULT_DUR_MS;
  }

  gMotors.stopAll();
  bool matched = true;
  if (strcmp(argTarget, "fl") == 0) {
    gMotors.setWheel(WHEEL_FL, pwm);
  } else if (strcmp(argTarget, "rl") == 0) {
    gMotors.setWheel(WHEEL_RL, pwm);
  } else if (strcmp(argTarget, "fr") == 0) {
    gMotors.setWheel(WHEEL_FR, pwm);
  } else if (strcmp(argTarget, "rr") == 0) {
    gMotors.setWheel(WHEEL_RR, pwm);
  } else if (strcmp(argTarget, "l") == 0) {
    gMotors.setLeftRight(pwm, 0);
  } else if (strcmp(argTarget, "r") == 0) {
    gMotors.setLeftRight(0, pwm);
  } else if (strcmp(argTarget, "all") == 0) {
    gMotors.setLeftRight(pwm, pwm);
  } else {
    matched = false;
  }

  if (!matched) {
    Serial.println(F("[ERR] Unknown wheel target. Use: fl rl fr rr l r all"));
    return;
  }

  gTracker.startManualTest(millis(), durMs);
  Serial.print(F("[TEST] target=")); Serial.print(argTarget);
  Serial.print(F(" pwm=")); Serial.print(pwm);
  Serial.print(F(" dur=")); Serial.print(durMs);
  Serial.println(F(" ms (send X or Space to stop early)"));
}

void handleScaleWheel(char* argWheel, char* argVal) {
  if (argWheel == nullptr || argVal == nullptr) {
    Serial.println(F("[ERR] Usage: SCALE <fl|rl|fr|rr> <0.50..1.30>"));
    return;
  }
  toLowerStr(argWheel);
  float val = constrain(static_cast<float>(atof(argVal)), 0.50f, 1.30f);
  WheelId w = WHEEL_COUNT;
  if (strcmp(argWheel, "fl") == 0) w = WHEEL_FL;
  else if (strcmp(argWheel, "rl") == 0) w = WHEEL_RL;
  else if (strcmp(argWheel, "fr") == 0) w = WHEEL_FR;
  else if (strcmp(argWheel, "rr") == 0) w = WHEEL_RR;

  if (w == WHEEL_COUNT) {
    Serial.println(F("[ERR] Unknown wheel. Use: fl rl fr rr"));
    return;
  }
  gMotors.params().scale[w] = val;
  Serial.print(F("[SCALE] ")); Serial.print(MotorDriver::wheelName(w));
  Serial.print(F(" = ")); Serial.println(val, 2);
}

void handleSetParam(char* key, char* valStr) {
  if (key == nullptr || valStr == nullptr) {
    Serial.println(F("[ERR] Usage: SET <key> <val> (send H to list keys)"));
    return;
  }

  toLowerStr(key);
  auto& mp0 = gMotors.params();
  auto& p = gTracker.params();
  auto& mp = gMission.params();

  if (strcmp(key, "kp") == 0) {
    p.kp = static_cast<float>(atof(valStr));
  } else if (strcmp(key, "kd") == 0) {
    p.kd = static_cast<float>(atof(valStr));
  } else if (strcmp(key, "ykp") == 0) {
    p.kpYawHold = static_cast<float>(atof(valStr));
  } else if (strcmp(key, "spd") == 0) {
    p.basePwm = constrain(atoi(valStr), 0, MOTOR_PWM_MAX);
  } else if (strcmp(key, "slow") == 0) {
    p.slowPwm = constrain(atoi(valStr), 0, MOTOR_PWM_MAX);
  } else if (strcmp(key, "stop") == 0) {
    p.frontStopMm = static_cast<uint16_t>(max(30, atoi(valStr)));
  } else if (strcmp(key, "rstop") == 0) {
    p.rearStopMm = static_cast<uint16_t>(max(30, atoi(valStr)));
  } else if (strcmp(key, "wall") == 0) {
    p.sideWallMaxMm = static_cast<uint16_t>(max(80, atoi(valStr)));
  } else if (strcmp(key, "side") == 0) {
    p.singleWallMm = static_cast<uint16_t>(max(30, atoi(valStr)));
  } else if (strcmp(key, "steer") == 0) {
    p.maxSteerPwm = constrain(atoi(valStr), 0, MOTOR_PWM_MAX);
  } else if (strcmp(key, "tpwm") == 0) {
    mp.turnMaxPwm = constrain(atoi(valStr), 0, MOTOR_PWM_MAX);
  } else if (strcmp(key, "tmin") == 0) {
    mp.turnMinPwm = constrain(atoi(valStr), 0, MOTOR_PWM_MAX);
  } else if (strcmp(key, "tkp") == 0) {
    mp.turnKp = static_cast<float>(atof(valStr));
  } else if (strcmp(key, "arc") == 0) {
    mp.turnArcRatio = constrain(static_cast<float>(atof(valStr)), 0.0f, 0.70f);
  } else if (strcmp(key, "tms") == 0) {
    mp.turnFallback90Ms = static_cast<uint16_t>(constrain(atoi(valStr), 100, 3000));
  } else if (strcmp(key, "cpwm") == 0) {
    mp.crossPwm = constrain(atoi(valStr), 0, MOTOR_PWM_MAX);
  } else if (strcmp(key, "cout") == 0) {
    mp.crossOuterWallMm = static_cast<uint16_t>(max(40, atoi(valStr)));
  } else if (strcmp(key, "cridge") == 0) {
    mp.crossRidgeMaxMm = static_cast<uint16_t>(max(100, atoi(valStr)));
  } else if (strcmp(key, "copen") == 0) {
    mp.crossOpenMinMm = static_cast<uint16_t>(max(150, atoi(valStr)));
  } else if (strcmp(key, "cmm") == 0) {
    mp.crossCenterMm = static_cast<uint16_t>(constrain(atoi(valStr), 20, 400));
  } else if (strcmp(key, "cmax") == 0) {
    mp.crossMaxDistMm = static_cast<uint16_t>(constrain(atoi(valStr), 300, 1200));
  } else if (strcmp(key, "ccent") == 0) {
    mp.crossCenterMs = static_cast<uint16_t>(constrain(atoi(valStr), 100, 4000));
  } else if (strcmp(key, "hdist") == 0) {
    mp.homeDistMm = static_cast<uint16_t>(constrain(atoi(valStr), 100, 2000));
  } else if (strcmp(key, "home") == 0) {
    mp.homeDriveMs = static_cast<uint16_t>(constrain(atoi(valStr), 300, 10000));
  } else if (strcmp(key, "vpid") == 0) {
    mp0.velPidEnabled = (atoi(valStr) != 0);
    gMotors.resetVelocityPid();
  } else if (strcmp(key, "vkp") == 0) {
    mp0.velKp = static_cast<float>(atof(valStr));
  } else if (strcmp(key, "vki") == 0) {
    mp0.velKi = static_cast<float>(atof(valStr));
  } else if (strcmp(key, "vmax") == 0) {
    mp0.maxSpeedMmS = constrain(static_cast<float>(atof(valStr)), 100.0f, 1000.0f);
  } else {
    Serial.println(F("[ERR] Unknown param key."));
    return;
  }

  Serial.print(F("[SET] ")); Serial.print(key);
  Serial.print(F(" = ")); Serial.println(valStr);
}

void runPinVoltageDiag() {
  cancelAll();
  PCICR &= ~(1 << PCIE2);
  for (uint8_t p = A8; p <= A15; ++p) {
    pinMode(p, INPUT);  // 关闭内部上拉，测真实引脚电平
  }

  uint16_t minVal[8];
  uint16_t maxVal[8];
  for (uint8_t i = 0; i < 8; ++i) {
    minVal[i] = 1023;
    maxVal[i] = 0;
  }

  Serial.println(F("[DIAG] Spinning all wheels at PWM=150 for 1000 ms, sampling A8..A15 voltage (no pullup)..."));
  gMotors.setLeftRight(150, 150);
  uint32_t startMs = millis();
  while ((millis() - startMs) < 1000UL) {
    for (uint8_t i = 0; i < 8; ++i) {
      uint16_t v = static_cast<uint16_t>(analogRead(A8 + i));
      if (v < minVal[i]) minVal[i] = v;
      if (v > maxVal[i]) maxVal[i] = v;
    }
  }
  gMotors.stopAll();

  Serial.println(F("--- A8..A15 Voltage Range (mV, no pullup) ---"));
  for (uint8_t i = 0; i < 8; ++i) {
    uint16_t minMv = static_cast<uint16_t>((static_cast<uint32_t>(minVal[i]) * 5000UL) / 1023UL);
    uint16_t maxMv = static_cast<uint16_t>((static_cast<uint32_t>(maxVal[i]) * 5000UL) / 1023UL);
    Serial.print(F("  A")); Serial.print(8 + i);
    Serial.print(F(": min=")); Serial.print(minMv);
    Serial.print(F(" mV  max=")); Serial.print(maxMv);
    Serial.println(F(" mV"));
  }
  gEncoders.begin();
}

void processCommandLine(char* line) {
  char* tok0 = strtok(line, " \t");
  if (tok0 == nullptr) {
    return;
  }
  toLowerStr(tok0);
  uint32_t nowMs = millis();

  if (strcmp(tok0, "x") == 0 || strcmp(tok0, "stop") == 0) {
    cancelAll();
    Serial.println(F("[CMD] STOP"));
  } else if (strcmp(tok0, "g") == 0) {
    cancelAll();
    gTracker.startForward(nowMs);
    Serial.println(F("[CMD] START FORWARD TRACKING"));
  } else if (strcmp(tok0, "b") == 0) {
    cancelAll();
    gTracker.startReverse(nowMs);
    Serial.println(F("[CMD] START REVERSE TRACKING"));
  } else if (strcmp(tok0, "d") == 0) {
    char* t1 = strtok(nullptr, " \t");
    if (t1 == nullptr) {
      Serial.println(F("[ERR] Usage: D <mm> (e.g. D 500 or D -300)"));
      return;
    }
    int16_t mm = static_cast<int16_t>(constrain(atoi(t1), -3000, 3000));
    cancelAll();
    gMission.startDriveDistanceMm(mm, nowMs);
  } else if (strcmp(tok0, "lt") == 0) {
    // 地面直线+转弯+直线综合测试：LT [第1段mm] [L|R] [第2段mm] [A]
    // 默认无参数：直行 800mm -> 原地右转 90° -> 直行 800mm（末尾加 A 则改为弧线 90° 转）
    char* t1 = strtok(nullptr, " \t");
    char* t2 = strtok(nullptr, " \t");
    char* t3 = strtok(nullptr, " \t");
    char* t4 = strtok(nullptr, " \t");
    int16_t d1 = (t1 != nullptr) ? static_cast<int16_t>(constrain(atoi(t1), 100, 3000)) : 800;
    bool turnLeft = false;
    if (t2 != nullptr) {
      toLowerStr(t2);
      turnLeft = (t2[0] == 'l');
    }
    int16_t d2 = (t3 != nullptr) ? static_cast<int16_t>(constrain(atoi(t3), 100, 3000)) : d1;
    int8_t arcSign = 0;
    if (t4 != nullptr) {
      toLowerStr(t4);
      if (t4[0] == 'a' || t4[0] == '1') {
        arcSign = +1;
      }
    }
    cancelAll();
    gMission.startLTest(d1, turnLeft, d2, arcSign, nowMs);
  } else if (strcmp(tok0, "tl") == 0) {
    cancelAll();
    gMission.startTurn90(true, 0, nowMs);
  } else if (strcmp(tok0, "tr") == 0) {
    cancelAll();
    gMission.startTurn90(false, 0, nowMs);
  } else if (strcmp(tok0, "al") == 0) {
    cancelAll();
    gMission.startTurn90(true, +1, nowMs);
  } else if (strcmp(tok0, "ar") == 0) {
    cancelAll();
    gMission.startTurn90(false, +1, nowMs);
  } else if (strcmp(tok0, "ul") == 0) {
    cancelAll();
    gMission.startLaneChange(true, nowMs);
  } else if (strcmp(tok0, "ur") == 0) {
    cancelAll();
    gMission.startLaneChange(false, nowMs);
  } else if (strcmp(tok0, "m") == 0) {
    cancelAll();
    char* t1 = strtok(nullptr, " \t");
    char* t2 = strtok(nullptr, " \t");
    uint8_t lanes = (t1 != nullptr) ? static_cast<uint8_t>(constrain(atoi(t1), 1, 14)) : 7;
    bool firstLeft = true;
    if (t2 != nullptr) {
      toLowerStr(t2);
      firstLeft = (t2[0] != 'r');
    }
    gMission.startSnakeMission(lanes, firstLeft, nowMs);
  } else if (strcmp(tok0, "f") == 0) {
    char* t1 = strtok(nullptr, " \t");
    uint8_t lanes = (t1 != nullptr) ? static_cast<uint8_t>(constrain(atoi(t1), 1, 14)) : AUTO_START_LANES;
    armFullMatch(lanes, 0, nowMs);
  } else if (strcmp(tok0, "save") == 0) {
    Storage::saveParams(gMotors.params(), gTracker.params(), gMission.params());
    Serial.println(F("[CFG] Saved motor, tracker & mission params to EEPROM."));
  } else if (strcmp(tok0, "load") == 0) {
    if (Storage::loadParams(gMotors.params(), gTracker.params(), gMission.params())) {
      gMotors.resetVelocityPid();
      Serial.println(F("[CFG] Reloaded params from EEPROM."));
    } else {
      Serial.println(F("[CFG] No valid saved params in EEPROM."));
    }
  } else if (strcmp(tok0, "resetcfg") == 0) {
    Storage::clearSavedParams();
    gMotors.params() = MotorParams{};
    gMotors.resetVelocityPid();
    gTracker.params() = TrackerParams{};
    gMission.params() = MissionParams{};
    Serial.println(F("[CFG] Cleared EEPROM and restored compiled defaults."));
  } else if (strcmp(tok0, "u") == 0) {
    gStreamTelemetry = !gStreamTelemetry;
    Serial.print(F("[CMD] Telemetry "));
    Serial.println(gStreamTelemetry ? F("ON") : F("OFF"));
  } else if (strcmp(tok0, "z") == 0) {
    gImu.zeroYaw();
    Serial.println(F("[IMU] Yaw zeroed -> 0.0 deg"));
  } else if (strcmp(tok0, "i") == 0) {
    gMission.abort();
    gImu.dumpRawHex(1000);
  } else if (strcmp(tok0, "cal") == 0) {
    gMission.abort();
    gImu.sendCalibrateImu(true);
    Serial.println(F("[IMU] Sent calibrate command (keep robot flat & still)."));
  } else if (strcmp(tok0, "ver") == 0) {
    gImu.sendRequestVersion();
    Serial.println(F("[IMU] Requested firmware version."));
  } else if (strcmp(tok0, "rate") == 0) {
    char* t1 = strtok(nullptr, " \t");
    if (t1 == nullptr) {
      Serial.println(F("[ERR] Usage: RATE <10..100>"));
      return;
    }
    uint8_t hz = static_cast<uint8_t>(constrain(atoi(t1), 10, 100));
    gImu.sendSetRateHz(hz);
    Serial.print(F("[IMU] Set output rate -> "));
    Serial.print(hz);
    Serial.println(F(" Hz"));
  } else if (strcmp(tok0, "baud") == 0) {
    char* t1 = strtok(nullptr, " \t");
    if (t1 == nullptr) {
      Serial.println(F("[ERR] Usage: BAUD <9600|115200>"));
      return;
    }
    uint32_t b = static_cast<uint32_t>(atol(t1));
    if (b < 1200UL || b > 921600UL) {
      Serial.println(F("[ERR] Invalid baud rate."));
      return;
    }
    gImu.setBaud(b);
    Serial.print(F("[IMU] Serial1 baud -> "));
    Serial.println(b);
  } else if (strcmp(tok0, "e") == 0) {
    printEncoderSummary();
  } else if (strcmp(tok0, "diag") == 0) {
    runPinVoltageDiag();
  } else if (strcmp(tok0, "ec") == 0) {
    gEncoders.reset();
    Serial.println(F("[ENC] Cleared all 4 wheel encoder counts to 0."));
  } else if (strcmp(tok0, "p") == 0) {
    printConfigSummary();
  } else if (strcmp(tok0, "h") == 0 || strcmp(tok0, "?") == 0) {
    printHelp();
  } else if (strcmp(tok0, "w") == 0) {
    char* t1 = strtok(nullptr, " \t");
    char* t2 = strtok(nullptr, " \t");
    char* t3 = strtok(nullptr, " \t");
    handleWheelTest(t1, t2, t3);
  } else if (strcmp(tok0, "sv") == 0) {
    // 舵机测试与角度控制：
    //   SV                  -> 依次小幅摆动测试 D32 -> D33 -> D34 (90->65->115->90°)
    //   SV TEST             -> 同上
    //   SV OFF              -> 释放全部舵机 PWM（断力保护）
    //   SV <32|33|34> TEST  -> 单独小幅摆动测试该舵机
    //   SV <32|33|34|all> <0..180> -> 转到指定角度
    char* t1 = strtok(nullptr, " \t");
    char* t2 = strtok(nullptr, " \t");
    if (t1 == nullptr) {
      gServos.startTestSweep(-1, nowMs);
      return;
    }
    toLowerStr(t1);
    if (strcmp(t1, "test") == 0) {
      gServos.startTestSweep(-1, nowMs);
      return;
    }
    if (strcmp(t1, "off") == 0 || strcmp(t1, "detach") == 0) {
      gServos.detachAll();
      Serial.println(F("[SERVO] 已断开 D32/D33/D34 全部 PWM 信号（断力释放）"));
      return;
    }
    if (strcmp(t1, "all") == 0) {
      if (t2 == nullptr) {
        gServos.startTestSweep(-1, nowMs);
      } else {
        uint8_t deg = static_cast<uint8_t>(constrain(atoi(t2), 0, 180));
        gServos.stopTest();
        gServos.setAll(deg);
        Serial.print(F("[SERVO] D32/D33/D34 全部 -> "));
        Serial.print(deg);
        Serial.println(F(" deg"));
      }
      return;
    }
    int8_t idx = ServoController::resolveIndex(static_cast<int16_t>(atoi(t1)));
    if (idx < 0) {
      Serial.println(F("[ERR] Usage: SV | SV OFF | SV <32|33|34|all> <0..180|test>"));
      return;
    }
    if (t2 == nullptr) {
      gServos.startTestSweep(idx, nowMs);
      return;
    }
    toLowerStr(t2);
    if (strcmp(t2, "test") == 0) {
      gServos.startTestSweep(idx, nowMs);
    } else {
      uint8_t deg = static_cast<uint8_t>(constrain(atoi(t2), 0, 180));
      gServos.stopTest();
      gServos.setAngle(static_cast<uint8_t>(idx), deg);
      Serial.print(F("[SERVO] D"));
      Serial.print(ServoController::pinOf(static_cast<uint8_t>(idx)));
      Serial.print(F(" -> "));
      Serial.print(deg);
      Serial.println(F(" deg"));
    }
  } else if (strcmp(tok0, "scale") == 0) {
    char* t1 = strtok(nullptr, " \t");
    char* t2 = strtok(nullptr, " \t");
    handleScaleWheel(t1, t2);
  } else if (strcmp(tok0, "set") == 0) {
    char* t1 = strtok(nullptr, " \t");
    char* t2 = strtok(nullptr, " \t");
    handleSetParam(t1, t2);
  } else {
    Serial.print(F("[ERR] Unknown command: "));
    Serial.println(tok0);
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    // 空格无需回车直接急停
    if (c == ' ' && gCmdLen == 0) {
      cancelAll();
      Serial.println(F("[E-STOP] Space received -> motors stopped."));
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (gCmdLen > 0) {
        gCmdBuf[gCmdLen] = '\0';
        processCommandLine(gCmdBuf);
        gCmdLen = 0;
      }
      continue;
    }

    if (gCmdLen < (kCmdBufSize - 1)) {
      gCmdBuf[gCmdLen++] = c;
    } else {
      gCmdLen = 0;
    }
  }
}

void pollStartButton(uint32_t nowMs) {
  if (!ENABLE_START_BUTTON) {
    return;
  }

  bool raw = digitalRead(PIN_START_BTN);
  if (raw != gBtnLastRaw) {
    gBtnLastRaw = raw;
    gBtnLastChangeMs = nowMs;
  }

  if ((nowMs - gBtnLastChangeMs) >= BTN_DEBOUNCE_MS && raw != gBtnStable) {
    gBtnStable = raw;
    if (gBtnStable == LOW) {
      // 按下触发：运行/倒计时中则急停，空闲则倒计时发车
      if (gArmedCountdown || gMission.isActive() || gTracker.status().mode != MODE_IDLE) {
        cancelAll();
        Serial.println(F("[BTN] Pressed while active -> EMERGENCY STOP."));
      } else {
        armFullMatch(AUTO_START_LANES, BTN_START_DELAY_MS, nowMs);
      }
    }
  }
}

void updateStatusLed(uint32_t nowMs) {
  bool ledOn = false;
  if (gArmedCountdown) {
    ledOn = ((nowMs / 80UL) % 2UL) == 0;  // 倒计时急闪
  } else if (gMission.isActive() || gTracker.status().mode != MODE_IDLE) {
    ledOn = true;  // 运行中常亮
  } else if (gImu.isOnline(nowMs)) {
    ledOn = ((nowMs / 500UL) % 2UL) == 0;  // 待机且 IMU 就绪：慢闪
  } else {
    ledOn = ((nowMs / 150UL) % 2UL) == 0;  // 待机但 IMU 未在线：较快闪提醒
  }
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

}  // namespace

// ============================================================================
// 上电初始化：先确保四路电机停机，再初始化传感器、加载 EEPROM 参数并打印配置
// ============================================================================
void setup() {
  gMotors.begin();  // 第一时间确保四路电机停机，防止上电乱动
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if (ENABLE_START_BUTTON) {
    pinMode(PIN_START_BTN, INPUT_PULLUP);
  }

  gUltrasonics.begin();
  gEncoders.begin();
  gImu.begin(IMU_BAUD_RATE);
  gServos.begin();
  gTracker.begin(&gMotors, &gUltrasonics, &gImu);
  gMission.begin(&gMotors, &gUltrasonics, &gEncoders, &gImu, &gTracker);

  Serial.begin(SERIAL_BAUD_RATE);
  delay(200);

  // 若 EEPROM 中存有上次调试保存的有效参数（魔数+版本号+校验和匹配），则自动覆盖默认值
  if (Storage::loadParams(gMotors.params(), gTracker.params(), gMission.params())) {
    Serial.println(F("[CFG] Loaded saved params from EEPROM."));
  } else {
    Serial.println(F("[CFG] Using compiled default params (send SAVE after tuning)."));
  }

  printHelp();
  printConfigSummary();

  if (AUTO_START_MODE == 1) {
    armAutoLTest(AUTO_START_DELAY_MS, millis());
  } else if (AUTO_START_MODE == 2) {
    armFullMatch(AUTO_START_LANES, AUTO_START_DELAY_MS, millis());
  } else if (AUTO_START_MODE == 3) {
    gArmedCountdown = true;
    gLaunchMode = 3;
    gLaunchAtMs = millis() + AUTO_START_DELAY_MS;
    Serial.print(F("[ARM] Standalone Servo Loop Test on D"));
    Serial.print(AUTO_SERVO_TEST_PIN);
    Serial.println(F(" starting in 1.5s... (motors stay IDLE)"));
  } else {
    Serial.println(F("[BOOT] Motors IDLE. Send 'U' monitor, 'W' test wheels, 'LT' L-test, 'F' full match."));
  }
}

// ============================================================================
// 主循环：所有控制均为非阻塞状态机推进，保持高响应急停与稳定的传感器节拍
// ============================================================================
void loop() {
  // 1. 最高优先级：处理串口命令与空格急停
  pollSerial();

  uint32_t nowMs = millis();
  // 2. 实体启动/急停按键检测
  pollStartButton(nowMs);

  // 3. 倒计时到点后自动切入预约任务（1=L形测试，2=全程比赛，3=脱机舵机循环测试）
  if (gArmedCountdown && static_cast<int32_t>(nowMs - gLaunchAtMs) >= 0) {
    gArmedCountdown = false;
    if (gLaunchMode == 1) {
      gMission.startLTest(
          AUTO_LTEST_LEG1_MM, AUTO_LTEST_TURN_LEFT, AUTO_LTEST_LEG2_MM, AUTO_LTEST_ARC_SIGN, nowMs);
    } else if (gLaunchMode == 3) {
      int8_t idx = ServoController::resolveIndex(AUTO_SERVO_TEST_PIN);
      if (idx >= 0) {
        gServos.startLoopTest(static_cast<uint8_t>(idx), nowMs);
      }
    } else {
      gMission.startFullMatch(gLaunchLanes, nowMs);
    }
  }

  // 4. 超声测距前先排空 Serial1，避免 64B 硬件接收缓冲在 pulseIn 期间溢出
  gImu.update(nowMs);

  // 5. 轮询 1 路超声波（每 25 ms 测 1 路，100 ms 完成 F->R->B->L 一轮）
  bool sampled = gUltrasonics.update(nowMs);

  nowMs = millis();
  // 6. 超声测距后立刻再排空一次 Serial1，并按 50 ms 节拍更新编码器速度/里程
  gImu.update(nowMs);
  gEncoders.update(nowMs);

  // 7. 若有新的超声采样（或处于手动限时测试），推进底盘循迹控制器
  if (sampled || gTracker.status().mode == MODE_MANUAL_TEST) {
    gTracker.update(nowMs);
  }

  // 8. 推进全程任务状态机与三路舵机非阻塞测试状态机
  gMission.update(nowMs);
  gServos.update(nowMs);

  // 9. 按 50 ms 节拍运行四轮独立速度 PI 闭环，拉齐四轮转速与抗毛毯阻力
  gMotors.updateVelocityLoop(gEncoders, nowMs);

  // 10. 更新板载 LED 运行指示灯（待机慢闪 / IMU离线快闪 / 倒计时急闪 / 运行常亮）
  updateStatusLed(nowMs);

  // 11. 可选：按 150 ms 周期打印实时遥测（串口发送 U 开关）
  if (gStreamTelemetry && (nowMs - gLastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
    gLastTelemetryMs = nowMs;
    printTelemetry(nowMs);
  }
}
