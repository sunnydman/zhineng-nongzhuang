#pragma once

#include <Arduino.h>
#include "config.h"

// ============================================================================
// 串口陀螺仪 / IMU 驱动（接扩展板左下角 "陀螺仪 R19 T18" -> Serial1）
// 支持两种协议（通过 config.h 中的 IMU_PROTOCOL 切换）：
//   1. 维特单轴高精度 HWT101CT-TTL（默认启用，9600 bps）：
//      - 0x55 0x52 [8B数据] [SUM] : Z 轴角速度 (deg/s)
//      - 0x55 0x53 [8B数据] [SUM] : Z 轴偏航角 (deg)
//      - 控制指令：FF AA 69 88 B5(解锁) -> FF AA 76 00 00(Z轴归零) / 03(速率) / 04(波特率)
//   2. 雅博六轴串口 IMU（备用，115200 bps，0x7E 0x23 协议）
// ============================================================================
class ImuSerial {
 public:
  void begin(uint32_t baud = IMU_BAUD_RATE);
  void setBaud(uint32_t baud);
  uint32_t baud() const { return baud_; }

  void update(uint32_t nowMs);

  // 将当前朝向置为 0° 软件参考零点（即时生效，不写传感器 Flash）
  void zeroYaw();

  // 是否在最近 IMU_ONLINE_TIMEOUT_MS 内收到过合法角度帧
  bool isOnline(uint32_t nowMs) const;

  // 相对零点的偏航角，范围 [-180, +180]，约定逆时针（左转）为正
  float yawDeg() const;
  float rollDeg() const { return rollDeg_; }
  float pitchDeg() const { return pitchDeg_; }
  float rawYawDeg() const { return rawYawDeg_; }
  float gyroZDegPerSec() const { return gyroZDegPerSec_; }

  uint32_t eulerPacketCount() const { return eulerPacketCount_; }
  uint32_t totalPacketCount() const { return totalPacketCount_; }
  uint8_t lastFuncCode() const { return lastFuncCode_; }

  // 下发传感器配置指令（自动适配当前 IMU_PROTOCOL）
  void sendSetRateHz(uint8_t hz);     // HWT101CT: 5/10/20/50/100 Hz (9600波特率下最高建议 20Hz)
  void sendCalibrateImu(bool start);  // HWT101CT: 硬件 Z 轴归零 (0x76)；雅博: 陀螺仪静止校准 (0x70)
  void sendRequestVersion();

  // 调试抓包：持续 durationMs 打印 Serial1 收到的原始十六进制字节，用于核对帧头/波特率
  void dumpRawHex(uint16_t durationMs);

  static float wrapAngleDeg(float deg);
  static const char* protocolName();

 private:
  enum RxState : uint8_t {
    RX_EXPECT_HEAD1 = 0,
    RX_EXPECT_HEAD2,
    RX_EXPECT_LENGTH,
    RX_EXPECT_FUNCTION,
    RX_COLLECT_DATA
  };

  void sendCmd2(uint8_t func, uint8_t p1, uint8_t p2);
  void sendWitReg(uint8_t reg, uint8_t dataL, uint8_t dataH);
  void parseByte(uint8_t b, uint32_t nowMs);
  void parseFrameData(uint8_t func, const uint8_t* data, uint16_t payloadLen, uint32_t nowMs);

  static int16_t toInt16LE(const uint8_t* p);
  static float toFloatLE(const uint8_t* p);

  uint32_t baud_ = IMU_BAUD_RATE;
  RxState rxState_ = RX_EXPECT_HEAD1;
  uint8_t frameLength_ = 0;
  uint8_t frameFunction_ = 0;
  uint8_t frameBuffer_[64] = {0};
  uint16_t frameIndex_ = 0;

  float rollDeg_ = 0.0f;
  float pitchDeg_ = 0.0f;
  float rawYawDeg_ = 0.0f;
  float zeroOffsetDeg_ = 0.0f;
  float gyroZDegPerSec_ = 0.0f;
  bool hasZero_ = false;

  uint32_t lastEulerMs_ = 0;
  uint32_t eulerPacketCount_ = 0;
  uint32_t totalPacketCount_ = 0;
  uint8_t lastFuncCode_ = 0;
};
