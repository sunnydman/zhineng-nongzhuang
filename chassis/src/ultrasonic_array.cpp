// ============================================================================
// 【文件作用】四路超声波防串扰分时测距与 3 点中值滤波实现
//   - 按 前(F) -> 右(R) -> 后(B) -> 左(L) 每 28ms 轮询 1 路（112ms 一整圈），
//     输出滤波后的毫米距离，供居中循迹、端头停车和扫田垄边沿使用。
// ============================================================================
#include "ultrasonic_array.h"

const UltrasonicArray::SensorPins UltrasonicArray::kPins[US_COUNT] = {
    {PIN_US_FRONT_TRIG, PIN_US_FRONT_ECHO},
    {PIN_US_REAR_TRIG,  PIN_US_REAR_ECHO},
    {PIN_US_LEFT_TRIG,  PIN_US_LEFT_ECHO},
    {PIN_US_RIGHT_TRIG, PIN_US_RIGHT_ECHO},
};

// 按 前 -> 右 -> 后 -> 左 交替触发，让对向墙面的声波有更长衰减时间
const UltrasonicId UltrasonicArray::kPollOrder[US_COUNT] = {
    US_FRONT,
    US_RIGHT,
    US_REAR,
    US_LEFT,
};

void UltrasonicArray::begin() {
  for (uint8_t i = 0; i < US_COUNT; ++i) {
    pinMode(kPins[i].trigPin, OUTPUT);
    digitalWrite(kPins[i].trigPin, LOW);
    pinMode(kPins[i].echoPin, INPUT);
  }
  clearFilters();
}

void UltrasonicArray::clearFilters() {
  for (uint8_t i = 0; i < US_COUNT; ++i) {
    readings_[i] = UltrasonicReading{};
    filters_[i] = FilterState{};
  }
  pollIndex_ = 0;
  lastSlotMs_ = 0;
}

bool UltrasonicArray::update(uint32_t nowMs) {
  if (lastSlotMs_ != 0 && (nowMs - lastSlotMs_) < US_SLOT_INTERVAL_MS) {
    return false;
  }
  lastSlotMs_ = nowMs;

  UltrasonicId id = kPollOrder[pollIndex_];
  pollIndex_ = (pollIndex_ + 1) % US_COUNT;
  lastSampled_ = id;

  uint16_t mm = measureOnceMm(id);
  auto& r = readings_[id];
  auto& f = filters_[id];
  r.rawMm = mm;
  r.lastUpdateMs = nowMs;
  f.sampledOnce = true;

  if (mm >= US_MIN_VALID_MM && mm <= US_MAX_VALID_MM) {
    f.missStreak = 0;
    f.window[f.head] = mm;
    f.head = (f.head + 1) % 3;
    if (f.count < 3) {
      f.count++;
    }

    if (f.count == 1) {
      r.filteredMm = f.window[0];
    } else if (f.count == 2) {
      r.filteredMm = static_cast<uint16_t>((static_cast<uint32_t>(f.window[0]) + f.window[1]) / 2U);
    } else {
      r.filteredMm = median3(f.window[0], f.window[1], f.window[2]);
    }
    r.valid = true;
  } else {
    if (f.missStreak < 255) {
      f.missStreak++;
    }
    if (f.missStreak >= US_STALE_LIMIT) {
      r.valid = false;
      r.filteredMm = 0;
      f.count = 0;
      f.head = 0;
    }
  }

  return true;
}

const UltrasonicReading& UltrasonicArray::get(UltrasonicId id) const {
  return readings_[id];
}

bool UltrasonicArray::allSampledOnce() const {
  for (uint8_t i = 0; i < US_COUNT; ++i) {
    if (!filters_[i].sampledOnce) {
      return false;
    }
  }
  return true;
}

uint16_t UltrasonicArray::measureOnceMm(UltrasonicId id) const {
  const auto& p = kPins[id];

  // 标准 HC-SR04 10 us 高电平触发脉冲
  digitalWrite(p.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(p.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(p.trigPin, LOW);

  unsigned long pulseUs = pulseIn(p.echoPin, HIGH, US_TIMEOUT_US);
  if (pulseUs == 0) {
    return 0;
  }

  // 声速约 343 m/s -> 单程距离 mm = pulseUs * 0.1715
  unsigned long distMm = (pulseUs * 1715UL) / 10000UL;
  if (distMm > 65535UL) {
    return 0;
  }
  return static_cast<uint16_t>(distMm);
}

uint16_t UltrasonicArray::median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    uint16_t t = a; a = b; b = t;
  }
  if (b > c) {
    uint16_t t = b; b = c; c = t;
  }
  if (a > b) {
    uint16_t t = a; a = b; b = t;
  }
  return b;
}

const char* UltrasonicArray::sensorName(UltrasonicId id) {
  switch (id) {
    case US_FRONT: return "F";
    case US_REAR:  return "B";
    case US_LEFT:  return "L";
    case US_RIGHT: return "R";
    default:       return "?";
  }
}
