// ============================================================================
// 【文件作用】双路 OpenMV 视觉串口接收头文件（声明接口，平时调参不用改）
//   - 负责经 Serial2 / Serial3 接收左右 OpenMV 发来的 $MV1 芒果识别帧。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

// ============================================================================
// 单路 OpenMV 串口接收与 MV1 协议解析器
// 对齐 openmv/当前版本/protocol.py:
//   $MV1,<seq>,<capture_ms>,<state>,<cx>,<cy>,<w>,<h>,<pixels>,<green_count>,<yellow_count>*<XX>\n
// 其中 *XX 为 '$' 与 '*' 之间所有 ASCII 字节的异或校验 (2 位大写十六进制)
// ============================================================================
struct OpenMvFrame {
  uint32_t seq         = 0;
  uint32_t captureMs   = 0;
  char state           = '?';  // 'U'暖机 / 'N'无目标 / 'W'确认中 / 'T'稳定锁定 / 'E'异常
  int16_t cx           = -1;
  int16_t cy           = -1;
  uint16_t w           = 0;
  uint16_t h           = 0;
  uint32_t pixels      = 0;
  uint8_t greenCount   = 0;
  uint8_t yellowCount  = 0;
  uint32_t rxMs        = 0;    // Mega 本地收到该帧的时间戳 (ms)
};

class OpenMvPort {
 public:
  void begin(HardwareSerial* port, const char* tag, uint32_t baud = OPENMV_BAUD_RATE);
  // 排空串口并解析新帧；若本轮收到至少 1 帧合法 MV1 返回 true
  bool update(uint32_t nowMs, bool logPackets = false);

  bool isOnline(uint32_t nowMs) const;
  const OpenMvFrame& latest() const { return latest_; }
  const char* tag() const { return tag_; }
  uint32_t rawByteCount() const { return rawBytes_; }
  uint32_t validPacketCount() const { return validPkts_; }
  uint32_t badChecksumCount() const { return badCrcPkts_; }
  uint32_t baud() const { return baud_; }

  void printStatus(uint32_t nowMs) const;

 private:
  void handleLine(uint32_t nowMs, bool logPackets);
  static int8_t hexNibble(char c);

  HardwareSerial* port_ = nullptr;
  const char* tag_ = "MV";
  uint32_t baud_ = OPENMV_BAUD_RATE;

  static constexpr size_t kLineBufSize = 96;
  char lineBuf_[kLineBufSize] = {0};
  size_t lineLen_ = 0;
  bool inFrame_ = false;

  OpenMvFrame latest_{};
  uint32_t rawBytes_ = 0;
  uint32_t validPkts_ = 0;
  uint32_t badCrcPkts_ = 0;
};

// 同时管理 Serial2 (R17/T16) 与 Serial3 (R15/T14) 两路 OpenMV 接口
class OpenMvManager {
 public:
  void begin(uint32_t baud = OPENMV_BAUD_RATE);
  void update(uint32_t nowMs);

  void setStreamLog(bool enabled) { streamLog_ = enabled; }
  bool streamLog() const { return streamLog_; }

  void printSummary(uint32_t nowMs) const;
  // 原始串口抓包：同时监听 Serial2 与 Serial3 持续 durationMs，打印收到的原始文本/HEX
  void dumpRawBoth(uint16_t durationMs);
  // 临时借用 Serial1 (R19/T18) @ 115200 测试单路 OpenMV（测前务必断开陀螺仪）
  void testOnSerial1(uint16_t durationMs, uint32_t restoreBaud);

  const OpenMvPort& port2() const { return mv2_; }
  const OpenMvPort& port3() const { return mv3_; }

 private:
  OpenMvPort mv2_;
  OpenMvPort mv3_;
  bool streamLog_ = false;
};
