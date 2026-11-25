#include "encoder_array.h"

#include <avr/interrupt.h>
#include <math.h>

namespace {

constexpr float kPi = 3.14159265f;

// 4 倍频正交格雷码查表：索引 = (prevAB << 2) | currAB
const int8_t kQuadTable[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

// 顺序对应 WheelId: 0=FL(A12/A13=PK4..5), 1=RL(A8/A9=PK0..1),
//                   2=FR(A14/A15=PK6..7), 3=RR(A10/A11=PK2..3)
volatile int32_t gIsrCounts[WHEEL_COUNT] = {0, 0, 0, 0};
volatile uint32_t gIsrPinEdges[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t gPrevPink = 0;

}  // namespace

ISR(PCINT2_vect) {
  uint8_t curr = PINK;
  uint8_t prev = gPrevPink;
  uint8_t diff = curr ^ prev;
  gPrevPink = curr;

  for (uint8_t b = 0; b < 8; ++b) {
    if (diff & (1U << b)) {
      gIsrPinEdges[b]++;
    }
  }

  // 新左后 RL: A8(bit0), A9(bit1)
  if (diff & 0x03) {
    uint8_t idx = static_cast<uint8_t>(((prev & 0x03) << 2) | (curr & 0x03));
    gIsrCounts[WHEEL_RL] += kQuadTable[idx];
  }
  // 新右后 RR: A10(bit2), A11(bit3)
  if (diff & 0x0C) {
    uint8_t idx = static_cast<uint8_t>((prev & 0x0C) | ((curr & 0x0C) >> 2));
    gIsrCounts[WHEEL_RR] += kQuadTable[idx];
  }
  // 新左前 FL: A12(bit4), A13(bit5)
  if (diff & 0x30) {
    uint8_t idx = static_cast<uint8_t>(((prev & 0x30) >> 2) | ((curr & 0x30) >> 4));
    gIsrCounts[WHEEL_FL] += kQuadTable[idx];
  }
  // 新右前 FR: A14(bit6), A15(bit7)
  if (diff & 0xC0) {
    uint8_t idx = static_cast<uint8_t>(((prev & 0xC0) >> 4) | ((curr & 0xC0) >> 6));
    gIsrCounts[WHEEL_FR] += kQuadTable[idx];
  }
}

void EncoderArray::begin() {
  // 全部启用内部上拉：兼容开漏霍尔输出，且悬空未接时不会误触发中断
  for (uint8_t p = A8; p <= A15; ++p) {
    pinMode(p, INPUT_PULLUP);
  }

  noInterrupts();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    gIsrCounts[i] = 0;
    snapshotCounts_[i] = 0;
    prevCounts_[i] = 0;
    speedMmPerSec_[i] = 0.0f;
  }
  for (uint8_t b = 0; b < 8; ++b) {
    gIsrPinEdges[b] = 0;
  }
  gPrevPink = PINK;
  PCIFR |= (1 << PCIF2);   // 清除可能挂起的中断标志
  PCMSK2 = 0xFF;           // 使能 A8-A15 全部 8 路引脚变化中断
  PCICR |= (1 << PCIE2);   // 开启 PCINT2 中断组
  interrupts();

  lastSampleMs_ = millis();
}

void EncoderArray::reset() {
  noInterrupts();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    gIsrCounts[i] = 0;
  }
  for (uint8_t b = 0; b < 8; ++b) {
    gIsrPinEdges[b] = 0;
  }
  interrupts();

  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    snapshotCounts_[i] = 0;
    prevCounts_[i] = 0;
    speedMmPerSec_[i] = 0.0f;
  }
  lastSampleMs_ = millis();
}

void EncoderArray::update(uint32_t nowMs) {
  uint32_t dtMs = nowMs - lastSampleMs_;
  if (dtMs < ENCODER_SAMPLE_MS) {
    return;
  }
  lastSampleMs_ = nowMs;

  int32_t raw[WHEEL_COUNT];
  noInterrupts();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    raw[i] = gIsrCounts[i];
  }
  interrupts();

  static const bool kRev[WHEEL_COUNT] = {
      ENCODER_FL_REVERSED,
      ENCODER_RL_REVERSED,
      ENCODER_FR_REVERSED,
      ENCODER_RR_REVERSED,
  };

  const float scaleMm = mmPerCount();
  const float dtSec = static_cast<float>(dtMs) * 0.001f;

  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    int32_t signedCnt = kRev[i] ? -raw[i] : raw[i];
    int32_t delta = signedCnt - prevCounts_[i];
    prevCounts_[i] = signedCnt;
    snapshotCounts_[i] = signedCnt;
    speedMmPerSec_[i] = (static_cast<float>(delta) * scaleMm) / dtSec;
  }
}

int32_t EncoderArray::counts(WheelId wheel) const {
  if (wheel >= WHEEL_COUNT) {
    return 0;
  }
  return snapshotCounts_[wheel];
}

float EncoderArray::distanceMm(WheelId wheel) const {
  if (wheel >= WHEEL_COUNT) {
    return 0.0f;
  }
  return static_cast<float>(snapshotCounts_[wheel]) * mmPerCount();
}

float EncoderArray::speedMmPerSec(WheelId wheel) const {
  if (wheel >= WHEEL_COUNT) {
    return 0.0f;
  }
  return speedMmPerSec_[wheel];
}

float EncoderArray::avgDistanceMm() const {
  float sum = 0.0f;
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    sum += distanceMm(static_cast<WheelId>(i));
  }
  return sum * 0.25f;
}

float EncoderArray::avgSpeedMmPerSec() const {
  float sum = 0.0f;
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    sum += speedMmPerSec_[i];
  }
  return sum * 0.25f;
}

bool EncoderArray::anyActivity() const {
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    if (snapshotCounts_[i] != 0) {
      return true;
    }
  }
  return false;
}

uint8_t EncoderArray::rawPink() const {
  return PINK;
}

uint32_t EncoderArray::rawPinEdges(uint8_t bitIndex) const {
  if (bitIndex >= 8) {
    return 0;
  }
  noInterrupts();
  uint32_t val = gIsrPinEdges[bitIndex];
  interrupts();
  return val;
}

float EncoderArray::mmPerCount() {
  if (ENCODER_COUNTS_PER_REV <= 1.0f) {
    return 0.0f;
  }
  return (kPi * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV;
}
