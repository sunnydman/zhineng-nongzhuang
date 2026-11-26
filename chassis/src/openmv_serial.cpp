// ============================================================================
// 【文件作用】双路 OpenMV 视觉串口接收与 $MV1 协议异或校验解析实现
//   - 并发接收 Serial2(16/17) 和 Serial3(14/15) 发来的芒果识别结果帧，
//     供后续接入“视觉识别 + 编码器提前量动态击打”使用。
// ============================================================================
#include "openmv_serial.h"

#include <stdlib.h>
#include <string.h>

void OpenMvPort::begin(HardwareSerial* port, const char* tag, uint32_t baud) {
  port_ = port;
  tag_ = tag;
  baud_ = baud;
  lineLen_ = 0;
  inFrame_ = false;
  latest_ = OpenMvFrame{};
  harvest_ = HarvestFrame{};
  rawBytes_ = 0;
  validPkts_ = 0;
  badCrcPkts_ = 0;
  if (port_ != nullptr) {
    port_->begin(baud_);
  }
}

bool OpenMvPort::update(uint32_t nowMs, bool logPackets) {
  if (port_ == nullptr) {
    return false;
  }

  bool gotFrame = false;
  while (port_->available() > 0) {
    char c = static_cast<char>(port_->read());
    rawBytes_++;

    if (c == '$') {
      inFrame_ = true;
      lineLen_ = 0;
      continue;
    }
    if (!inFrame_) {
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (lineLen_ > 0) {
        lineBuf_[lineLen_] = '\0';
        uint32_t before = validPkts_;
        handleLine(nowMs, logPackets);
        if (validPkts_ > before) {
          gotFrame = true;
        }
      }
      inFrame_ = false;
      lineLen_ = 0;
      continue;
    }

    if (lineLen_ < (kLineBufSize - 1)) {
      lineBuf_[lineLen_++] = c;
    } else {
      inFrame_ = false;
      lineLen_ = 0;
    }
  }
  return gotFrame;
}

int8_t OpenMvPort::hexNibble(char c) {
  if (c >= '0' && c <= '9') return static_cast<int8_t>(c - '0');
  if (c >= 'A' && c <= 'F') return static_cast<int8_t>(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return static_cast<int8_t>(c - 'a' + 10);
  return -1;
}

void OpenMvPort::handleLine(uint32_t nowMs, bool logPackets) {
  // 期望格式（已去掉 '$' 与 '\n'）：
  //   MV1,<seq>,<ms>,<state>,<cx>,<cy>,<w>,<h>,<pixels>,<gCnt>,<yCnt>*<XX>
  char* star = strchr(lineBuf_, '*');
  if (star == nullptr || star[1] == '\0' || star[2] == '\0') {
    badCrcPkts_++;
    return;
  }

  int8_t hi = hexNibble(star[1]);
  int8_t lo = hexNibble(star[2]);
  if (hi < 0 || lo < 0) {
    badCrcPkts_++;
    return;
  }
  uint8_t rxXor = static_cast<uint8_t>((hi << 4) | lo);

  uint8_t calcXor = 0;
  for (const char* p = lineBuf_; p < star; ++p) {
    calcXor ^= static_cast<uint8_t>(*p);
  }
  if (calcXor != rxXor) {
    badCrcPkts_++;
    return;
  }

  *star = '\0';  // 截断 '*XX'，便于解析 CSV 字段
  const bool stopMode = strncmp(lineBuf_, "HV2,", 4) == 0;
  if (stopMode || strncmp(lineBuf_, "HV1,", 4) == 0) {
    const char* fields = lineBuf_ + 4;
    char* tail = nullptr;
    const unsigned long seq = strtoul(fields, &tail, 10);
    if (*fields >= '0' && *fields <= '9' && seq <= 65535UL && star[3] == '\0' &&
        strlen(tail) == 6 && tail[0] == ',' && tail[2] == ',' && tail[4] == ',' &&
        (tail[1] == '0' || tail[1] == '1') && (tail[5] == '0' || tail[5] == '1') &&
        (tail[3] == 'N' || tail[3] == 'H' || tail[3] == 'L') &&
        (tail[5] == '0' || tail[3] != 'N') && (tail[1] == '1' || tail[3] == 'N')) {
      if (harvest_.received && harvest_.stopMode == stopMode && harvest_.seq == seq) {
        return;
      }
      harvest_.received = true;
      harvest_.seq = static_cast<uint16_t>(seq);
      harvest_.rxMs = nowMs;
      harvest_.nearFruit = tail[1] == '1';
      harvest_.target = tail[3];
      harvest_.aligned = tail[5] == '1';
      harvest_.stopMode = stopMode;
    }
    return;
  }
  char* fields[11];
  uint8_t count = 0;
  char* tok = strtok(lineBuf_, ",");
  while (tok != nullptr && count < 11) {
    fields[count++] = tok;
    tok = strtok(nullptr, ",");
  }
  if (count != 11 || strcmp(fields[0], "MV1") != 0) {
    badCrcPkts_++;
    return;
  }

  latest_.seq         = static_cast<uint32_t>(strtoul(fields[1], nullptr, 10));
  latest_.captureMs   = static_cast<uint32_t>(strtoul(fields[2], nullptr, 10));
  latest_.state       = fields[3][0];
  latest_.cx          = static_cast<int16_t>(atoi(fields[4]));
  latest_.cy          = static_cast<int16_t>(atoi(fields[5]));
  latest_.w           = static_cast<uint16_t>(atoi(fields[6]));
  latest_.h           = static_cast<uint16_t>(atoi(fields[7]));
  latest_.pixels      = static_cast<uint32_t>(strtoul(fields[8], nullptr, 10));
  latest_.greenCount  = static_cast<uint8_t>(atoi(fields[9]));
  latest_.yellowCount = static_cast<uint8_t>(atoi(fields[10]));
  latest_.rxMs        = nowMs;
  validPkts_++;

  if (logPackets) {
    Serial.print('['); Serial.print(tag_); Serial.print(F("] #"));
    Serial.print(latest_.seq);
    Serial.print(F(" st=")); Serial.print(latest_.state);
    Serial.print(F(" pos=(")); Serial.print(latest_.cx);
    Serial.print(','); Serial.print(latest_.cy);
    Serial.print(F(") box=")); Serial.print(latest_.w);
    Serial.print('x'); Serial.print(latest_.h);
    Serial.print(F(" px=")); Serial.print(latest_.pixels);
    Serial.print(F(" G=")); Serial.print(latest_.greenCount);
    Serial.print(F(" Y=")); Serial.println(latest_.yellowCount);
  }
}

bool OpenMvPort::isOnline(uint32_t nowMs) const {
  return (validPkts_ > 0) && ((nowMs - latest_.rxMs) <= OPENMV_ONLINE_TIMEOUT_MS);
}

void OpenMvPort::printStatus(uint32_t nowMs) const {
  Serial.print(F("  ")); Serial.print(tag_);
  Serial.print(F(" @")); Serial.print(baud_);
  Serial.print(F(": online=")); Serial.print(isOnline(nowMs) ? F("YES") : F("NO"));
  Serial.print(F(" rawB=")); Serial.print(rawBytes_);
  Serial.print(F(" okPkts=")); Serial.print(validPkts_);
  Serial.print(F(" badCrc=")); Serial.print(badCrcPkts_);
  if (validPkts_ > 0) {
    Serial.print(F(" | st=")); Serial.print(latest_.state);
    Serial.print(F(" (")); Serial.print(latest_.cx);
    Serial.print(','); Serial.print(latest_.cy);
    Serial.print(F(") G=")); Serial.print(latest_.greenCount);
    Serial.print(F(" Y=")); Serial.print(latest_.yellowCount);
  }
  Serial.println();
}

void OpenMvManager::begin(uint32_t baud) {
  mv2_.begin(&Serial2, "MV2(R17/T16)", baud);
  mv3_.begin(&Serial3, "MV3(R15/T14)", baud);
  streamLog_ = false;
}

void OpenMvManager::update(uint32_t nowMs) {
  mv2_.update(nowMs, streamLog_);
  mv3_.update(nowMs, streamLog_);
}

void OpenMvManager::printSummary(uint32_t nowMs) const {
  Serial.println(F("--- OpenMV Dual UART Status (MV1 Protocol, 115200 8N1) ---"));
  mv2_.printStatus(nowMs);
  mv3_.printStatus(nowMs);
}

void OpenMvManager::dumpRawBoth(uint16_t durationMs) {
  Serial.print(F("[MVRAW] 同时抓包 Serial2(R17/T16) 与 Serial3(R15/T14) @ "));
  Serial.print(OPENMV_BAUD_RATE);
  Serial.print(F(" bps，持续 "));
  Serial.print(durationMs);
  Serial.println(F(" ms..."));

  // 先清空旧残留缓冲，看真实实时输入
  while (Serial2.available() > 0) Serial2.read();
  while (Serial3.available() > 0) Serial3.read();

  uint32_t startMs = millis();
  uint16_t cnt2 = 0;
  uint16_t cnt3 = 0;
  char buf2[160];
  char buf3[160];

  while ((millis() - startMs) < durationMs) {
    while (Serial2.available() > 0 && cnt2 < (sizeof(buf2) - 1)) {
      buf2[cnt2++] = static_cast<char>(Serial2.read());
    }
    while (Serial3.available() > 0 && cnt3 < (sizeof(buf3) - 1)) {
      buf3[cnt3++] = static_cast<char>(Serial3.read());
    }
  }
  buf2[cnt2] = '\0';
  buf3[cnt3] = '\0';

  Serial.print(F("  [Serial2 R17/T16] 收到 ")); Serial.print(cnt2); Serial.println(F(" bytes:"));
  if (cnt2 > 0) {
    Serial.println(buf2);
  }
  Serial.print(F("  [Serial3 R15/T14] 收到 ")); Serial.print(cnt3); Serial.println(F(" bytes:"));
  if (cnt3 > 0) {
    Serial.println(buf3);
  }
  if (cnt2 == 0 && cnt3 == 0) {
    Serial.println(F("[MVRAW] 两路均 0 字节！请核对：(1) OpenMV config.py 的 UART_ENABLED=True；(2) P4(TX)->R17或R15；(3) 共地 GND。"));
  }
}

void OpenMvManager::testOnSerial1(uint16_t durationMs, uint32_t restoreBaud) {
  Serial.print(F("[MV1TEST] 临时切换 Serial1(R19/T18) -> "));
  Serial.print(OPENMV_BAUD_RATE);
  Serial.print(F(" bps 监听 OpenMV，持续 "));
  Serial.print(durationMs);
  Serial.println(F(" ms..."));

  Serial1.end();
  OpenMvPort tmp;
  tmp.begin(&Serial1, "MV1_TEST(R19/T18)", OPENMV_BAUD_RATE);
  while (Serial1.available() > 0) Serial1.read();

  uint32_t startMs = millis();
  while ((millis() - startMs) < durationMs) {
    tmp.update(millis(), true);
  }
  tmp.printStatus(millis());
  if (tmp.rawByteCount() == 0) {
    Serial.println(F("[MV1TEST] 收到 0 字节！请核对：(1) OpenMV 已通电且安全弹出重启；(2) P4(TX)->R19；(3) GND 共地；(4) 陀螺仪已断开。"));
  } else if (tmp.validPacketCount() == 0) {
    Serial.println(F("[MV1TEST] 收到字节但未解析出合法 $MV1 帧，抓 1 秒原始字符供排查："));
    uint32_t t0 = millis();
    uint16_t n = 0;
    while ((millis() - t0) < 1000UL && n < 160) {
      if (Serial1.available() > 0) {
        char c = static_cast<char>(Serial1.read());
        if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') {
          Serial.print(c);
        } else {
          Serial.print('['); Serial.print(static_cast<uint8_t>(c), HEX); Serial.print(']');
        }
        n++;
      }
    }
    Serial.println();
  }

  Serial1.end();
  Serial1.begin(restoreBaud);
}
