// ============================================================================
// 【文件作用】维特 HWT101CT-TTL 串口陀螺仪协议解析实现（Serial1: R19/T18）
//   - 解析 0x55 0x52(Z轴角速度) 与 0x55 0x53(偏航角 Yaw) 数据帧，提供软件归零、
//     硬件 Z 归零 (CAL) 与改回传频率 (RATE) 指令（保留雅博协议备用）。
// ============================================================================
#include "imu_serial.h"

#include <math.h>
#include <string.h>

namespace {
constexpr float kRadToDeg = 57.2957795f;

// 维特 HWT101CT-TTL 固定 11 字节协议常量
constexpr uint8_t kWitHead         = 0x55;
constexpr uint8_t kWitTypeGyro     = 0x52;  // 角速度帧：buf[6..7] 为 Wz (±2000 deg/s)
constexpr uint8_t kWitTypeAngle    = 0x53;  // 角度帧：  buf[6..7] 为 Yaw (±180 deg)
constexpr uint8_t kWitTypeRegReply = 0x5F;  // 寄存器读取返回帧
constexpr uint8_t kWitFrameLen     = 11;
}  // namespace

const char* ImuSerial::protocolName() {
  return (IMU_PROTOCOL == IMU_PROTO_WIT_HWT101) ? "HWT101CT(0x55)" : "Yahboom(0x7E23)";
}

void ImuSerial::begin(uint32_t baud) {
  baud_ = baud;
  Serial1.begin(baud_);
  rxState_ = RX_EXPECT_HEAD1;
  frameLength_ = 0;
  frameFunction_ = 0;
  frameIndex_ = 0;
  rollDeg_ = 0.0f;
  pitchDeg_ = 0.0f;
  rawYawDeg_ = 0.0f;
  zeroOffsetDeg_ = 0.0f;
  gyroZDegPerSec_ = 0.0f;
  hasZero_ = false;
  lastEulerMs_ = 0;
  eulerPacketCount_ = 0;
  totalPacketCount_ = 0;
  lastFuncCode_ = 0;
}

void ImuSerial::setBaud(uint32_t baud) {
  Serial1.end();
  begin(baud);
}

void ImuSerial::update(uint32_t nowMs) {
  while (Serial1.available() > 0) {
    parseByte(static_cast<uint8_t>(Serial1.read()), nowMs);
  }
}

void ImuSerial::zeroYaw() {
  zeroOffsetDeg_ = rawYawDeg_;
  hasZero_ = true;
}

bool ImuSerial::isOnline(uint32_t nowMs) const {
  return (eulerPacketCount_ > 0) && ((nowMs - lastEulerMs_) <= IMU_ONLINE_TIMEOUT_MS);
}

float ImuSerial::yawDeg() const {
  float rel = wrapAngleDeg(rawYawDeg_ - zeroOffsetDeg_);
  return IMU_YAW_INVERT ? -rel : rel;
}

float ImuSerial::wrapAngleDeg(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg <= -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

void ImuSerial::sendCmd2(uint8_t func, uint8_t p1, uint8_t p2) {
  uint8_t frame[7];
  frame[0] = IMU_FRAME_HEAD1;
  frame[1] = IMU_FRAME_HEAD2;
  frame[2] = 0x07;
  frame[3] = func;
  frame[4] = p1;
  frame[5] = p2;
  frame[6] = static_cast<uint8_t>(
      frame[0] + frame[1] + frame[2] + frame[3] + frame[4] + frame[5]);
  Serial1.write(frame, sizeof(frame));
}

void ImuSerial::sendWitReg(uint8_t reg, uint8_t dataL, uint8_t dataH) {
  const uint8_t frame[5] = {0xFF, 0xAA, reg, dataL, dataH};
  Serial1.write(frame, sizeof(frame));
}

void ImuSerial::sendSetRateHz(uint8_t hz) {
  if (IMU_PROTOCOL == IMU_PROTO_WIT_HWT101) {
    // 维特 0x03 寄存器速率码：0x05=5Hz, 0x06=10Hz, 0x07=20Hz, 0x08=50Hz, 0x09=100Hz
    uint8_t code = 0x06;  // 10 Hz
    uint8_t actualHz = 10;
    if (hz <= 5) {
      code = 0x05; actualHz = 5;
    } else if (hz <= 15) {
      code = 0x06; actualHz = 10;
    } else if (hz <= 35) {
      code = 0x07; actualHz = 20;
    } else if (hz <= 75) {
      code = 0x08; actualHz = 50;
    } else {
      code = 0x09; actualHz = 100;
    }

    if (baud_ <= 9600UL && actualHz > 20) {
      Serial.println(F("[WARN] HWT101CT 在 9600 bps 下最高稳定支持 20 Hz，已自动限幅到 20 Hz！"));
      code = 0x07;
      actualHz = 20;
    }

    sendWitReg(0x69, 0x88, 0xB5);  // 解锁寄存器
    delay(40);
    sendWitReg(0x03, code, 0x00);  // 写入回传速率
    delay(40);
    sendWitReg(0x00, 0x00, 0x00);  // 掉电保存
    Serial.print(F("[IMU] HWT101CT 回传速率已设为 "));
    Serial.print(actualHz);
    Serial.println(F(" Hz (已写入保存)"));
    return;
  }

  hz = constrain(hz, static_cast<uint8_t>(10), static_cast<uint8_t>(100));
  sendCmd2(IMU_FUNC_SET_RATE, hz, 0x5F);
}

void ImuSerial::sendCalibrateImu(bool start) {
  if (IMU_PROTOCOL == IMU_PROTO_WIT_HWT101) {
    if (!start) {
      return;
    }
    // 维特单轴 HWT101CT-TTL 官方硬件 Z 轴归零指令：解锁 -> 0x76 归零
    sendWitReg(0x69, 0x88, 0xB5);
    delay(40);
    sendWitReg(0x76, 0x00, 0x00);
    delay(40);
    zeroOffsetDeg_ = 0.0f;
    rawYawDeg_ = 0.0f;
    hasZero_ = true;
    Serial.println(F("[IMU] 已向 HWT101CT 发送硬件 Z 轴归零指令 (FF AA 76 00 00)"));
    return;
  }

  sendCmd2(IMU_FUNC_CALIB_IMU, start ? 0x01 : 0x00, 0x5F);
}

void ImuSerial::sendRequestVersion() {
  if (IMU_PROTOCOL == IMU_PROTO_WIT_HWT101) {
    // 读取版本号寄存器 0x2E（返回 0x55 0x5F 帧）
    sendWitReg(0x27, 0x2E, 0x00);
    return;
  }
  sendCmd2(IMU_FUNC_REQ_VERSION, 0x01, 0x00);
}

void ImuSerial::dumpRawHex(uint16_t durationMs) {
  Serial.print(F("[IMU] Sniffing Serial1 @ "));
  Serial.print(baud_);
  Serial.print(F(" bps for "));
  Serial.print(durationMs);
  Serial.println(F(" ms..."));

  uint32_t startMs = millis();
  uint16_t count = 0;
  while ((millis() - startMs) < durationMs && count < 128) {
    if (Serial1.available() > 0) {
      uint8_t b = static_cast<uint8_t>(Serial1.read());
      if (b < 0x10) {
        Serial.print('0');
      }
      Serial.print(b, HEX);
      Serial.print(' ');
      count++;
      if ((count % 16) == 0) {
        Serial.println();
      }
    }
  }
  if ((count % 16) != 0) {
    Serial.println();
  }
  if (count == 0) {
    Serial.println(F("[IMU] No bytes received on Serial1 (check TX->R19, RX->T18, power, GND, baud)."));
  } else {
    Serial.print(F("[IMU] Captured "));
    Serial.print(count);
    Serial.println(F(" bytes."));
  }
  rxState_ = RX_EXPECT_HEAD1;
  frameIndex_ = 0;
}

void ImuSerial::parseByte(uint8_t currentByte, uint32_t nowMs) {
  if (IMU_PROTOCOL == IMU_PROTO_WIT_HWT101) {
    // 维特 11 字节定长流解析（带滑窗重同步：就算载荷内出现 0x55 也能在 1 帧内自动重对齐）
    if (frameIndex_ == 0) {
      if (currentByte != kWitHead) {
        return;
      }
      frameBuffer_[frameIndex_++] = currentByte;
      return;
    }

    if (frameIndex_ == 1) {
      // 第二字节应为维特有效包类型 (0x50..0x5F)，否则若本字节就是 0x55 则顺延当新帧头
      if (currentByte >= 0x50 && currentByte <= 0x5F) {
        frameBuffer_[frameIndex_++] = currentByte;
      } else if (currentByte == kWitHead) {
        frameBuffer_[0] = kWitHead;
        frameIndex_ = 1;
      } else {
        frameIndex_ = 0;
      }
      return;
    }

    frameBuffer_[frameIndex_++] = currentByte;
    if (frameIndex_ < kWitFrameLen) {
      return;
    }

    // 收满 11 字节，核对前 10 字节累加和
    uint8_t calcSum = 0;
    for (uint8_t i = 0; i < (kWitFrameLen - 1); ++i) {
      calcSum = static_cast<uint8_t>(calcSum + frameBuffer_[i]);
    }

    if (calcSum == frameBuffer_[kWitFrameLen - 1]) {
      parseFrameData(frameBuffer_[1], &frameBuffer_[2], 8, nowMs);
      frameIndex_ = 0;
      return;
    }

    // 校验失败：在缓冲内寻找下一个 0x55 重新滑窗对齐，避免丢失后续有效帧
    uint8_t nextHead = kWitFrameLen;
    for (uint8_t i = 1; i < kWitFrameLen; ++i) {
      if (frameBuffer_[i] == kWitHead) {
        nextHead = i;
        break;
      }
    }
    if (nextHead < kWitFrameLen) {
      uint8_t remain = kWitFrameLen - nextHead;
      memmove(frameBuffer_, &frameBuffer_[nextHead], remain);
      frameIndex_ = remain;
    } else {
      frameIndex_ = 0;
    }
    return;
  }

  // -------------------- 备用：原雅博 0x7E 0x23 协议解析 --------------------
  switch (rxState_) {
    case RX_EXPECT_HEAD1:
      rxState_ = (currentByte == IMU_FRAME_HEAD1) ? RX_EXPECT_HEAD2 : RX_EXPECT_HEAD1;
      break;

    case RX_EXPECT_HEAD2:
      if (currentByte == IMU_FRAME_HEAD2) {
        rxState_ = RX_EXPECT_LENGTH;
      } else if (currentByte == IMU_FRAME_HEAD1) {
        rxState_ = RX_EXPECT_HEAD2;
      } else {
        rxState_ = RX_EXPECT_HEAD1;
      }
      break;

    case RX_EXPECT_LENGTH:
      frameLength_ = currentByte;
      rxState_ = RX_EXPECT_FUNCTION;
      break;

    case RX_EXPECT_FUNCTION:
      frameFunction_ = currentByte;
      frameIndex_ = 0;
      rxState_ = RX_COLLECT_DATA;
      break;

    case RX_COLLECT_DATA: {
      uint16_t dataLen = (frameLength_ >= 4) ? static_cast<uint16_t>(frameLength_ - 4) : 0;
      if (dataLen == 0 || dataLen > sizeof(frameBuffer_)) {
        rxState_ = RX_EXPECT_HEAD1;
        break;
      }

      frameBuffer_[frameIndex_++] = currentByte;
      if (frameIndex_ >= dataLen) {
        uint8_t calcSum = static_cast<uint8_t>(
            IMU_FRAME_HEAD1 + IMU_FRAME_HEAD2 + frameLength_ + frameFunction_);
        for (uint16_t i = 0; i < dataLen - 1; ++i) {
          calcSum = static_cast<uint8_t>(calcSum + frameBuffer_[i]);
        }

        uint8_t rxSum = frameBuffer_[dataLen - 1];
        if (calcSum == rxSum) {
          parseFrameData(frameFunction_, frameBuffer_, static_cast<uint16_t>(dataLen - 1), nowMs);
        }
        rxState_ = RX_EXPECT_HEAD1;
      }
    } break;

    default:
      rxState_ = RX_EXPECT_HEAD1;
      break;
  }
}

void ImuSerial::parseFrameData(uint8_t func, const uint8_t* data, uint16_t payloadLen, uint32_t nowMs) {
  totalPacketCount_++;
  lastFuncCode_ = func;

  if (IMU_PROTOCOL == IMU_PROTO_WIT_HWT101) {
    if (func == kWitTypeAngle && payloadLen >= 8) {
      // 维特角度帧 0x55 0x53：data[0..1]=Roll, data[2..3]=Pitch, data[4..5]=Yaw (±180°)
      constexpr float kAngleScale = 180.0f / 32768.0f;
      rollDeg_ = static_cast<float>(toInt16LE(&data[0])) * kAngleScale;
      pitchDeg_ = static_cast<float>(toInt16LE(&data[2])) * kAngleScale;
      rawYawDeg_ = wrapAngleDeg(static_cast<float>(toInt16LE(&data[4])) * kAngleScale);

      if (!hasZero_) {
        zeroOffsetDeg_ = rawYawDeg_;
        hasZero_ = true;
      }
      lastEulerMs_ = nowMs;
      eulerPacketCount_++;
    } else if (func == kWitTypeGyro && payloadLen >= 8) {
      // 维特角速度帧 0x55 0x52：data[4..5]=Wz (±2000 deg/s)，实测正负与 Yaw 一致
      constexpr float kGyroScale = 2000.0f / 32768.0f;
      float gz = static_cast<float>(toInt16LE(&data[4])) * kGyroScale;
      gyroZDegPerSec_ = IMU_YAW_INVERT ? -gz : gz;
    } else if (func == kWitTypeRegReply && payloadLen >= 2) {
      uint16_t ver = static_cast<uint16_t>(toInt16LE(&data[0]));
      Serial.print(F("[IMU] HWT101CT RegReply val=0x"));
      Serial.println(ver, HEX);
    }
    return;
  }

  // -------------------- 备用：原雅博协议数据帧解析 --------------------
  if (func == IMU_FUNC_EULER && payloadLen >= 12) {
    float r = toFloatLE(&data[0]);
    float p = toFloatLE(&data[4]);
    float y = toFloatLE(&data[8]);
    if (isnan(r) || isnan(p) || isnan(y) || isinf(r) || isinf(p) || isinf(y)) {
      return;
    }

    const float scale = IMU_EULER_IS_RADIANS ? kRadToDeg : 1.0f;
    rollDeg_ = r * scale;
    pitchDeg_ = p * scale;
    rawYawDeg_ = wrapAngleDeg(y * scale);

    if (!hasZero_) {
      zeroOffsetDeg_ = rawYawDeg_;
      hasZero_ = true;
    }
    lastEulerMs_ = nowMs;
    eulerPacketCount_++;
  } else if (func == IMU_FUNC_RAW_ACCEL && payloadLen >= 12) {
    constexpr float kGyroDegPerSec = 2000.0f / 32767.0f;
    float gz = static_cast<float>(toInt16LE(&data[10])) * kGyroDegPerSec;
    gyroZDegPerSec_ = IMU_YAW_INVERT ? -gz : gz;
  } else if (func == IMU_FUNC_VERSION && payloadLen >= 3) {
    Serial.print(F("[IMU] Firmware v"));
    Serial.print(data[0]); Serial.print('.');
    Serial.print(data[1]); Serial.print('.');
    Serial.println(data[2]);
  } else if (func == IMU_FUNC_RETURN_STATE && payloadLen >= 2) {
    Serial.print(F("[IMU] Return func=0x"));
    Serial.print(data[0], HEX);
    Serial.print(F(" state="));
    Serial.println(data[1] ? F("OK") : F("FAIL"));
  }
}

int16_t ImuSerial::toInt16LE(const uint8_t* p) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

float ImuSerial::toFloatLE(const uint8_t* p) {
  float val = 0.0f;
  memcpy(&val, p, sizeof(float));
  return val;
}
