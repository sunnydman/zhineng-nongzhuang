// ============================================================================
// 【文件作用】EEPROM 掉电参数读写实现（魔数 + 版本号 kVersion + 校验和）
//   - 若之前在串口发过 SAVE，开机时会自动加载 EEPROM 参数覆盖 config.h 默认值；
//   - 改了 config.h 默认值想强制生效时，把下方 kVersion 数字加 1 即可。
// ============================================================================
#include "storage.h"

#include <EEPROM.h>

namespace Storage {
namespace {

constexpr int kEepromAddr = 0;
constexpr uint32_t kMagic = 0x4D475431UL;  // "MGT1"
constexpr uint16_t kVersion = 27;

struct PersistedBlock {
  uint32_t magic;
  uint16_t version;
  MotorParams motor;
  TrackerParams tracker;
  MissionParams mission;
  uint16_t checksum;
};

uint16_t computeChecksum(const PersistedBlock& blk) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&blk);
  const size_t len = sizeof(PersistedBlock) - sizeof(uint16_t);
  uint16_t sum1 = 0x5A;
  uint16_t sum2 = 0xA5;
  for (size_t i = 0; i < len; ++i) {
    sum1 = static_cast<uint16_t>((sum1 + bytes[i]) % 255U);
    sum2 = static_cast<uint16_t>((sum2 + sum1) % 255U);
  }
  return static_cast<uint16_t>((sum2 << 8) | sum1);
}

}  // namespace

bool loadParams(MotorParams& motor, TrackerParams& tracker, MissionParams& mission) {
  PersistedBlock blk{};
  EEPROM.get(kEepromAddr, blk);
  if (blk.magic != kMagic || blk.version != kVersion) {
    return false;
  }
  if (blk.checksum != computeChecksum(blk)) {
    return false;
  }
  motor = blk.motor;
  tracker = blk.tracker;
  mission = blk.mission;
  return true;
}

void saveParams(const MotorParams& motor, const TrackerParams& tracker, const MissionParams& mission) {
  PersistedBlock blk{};
  blk.magic = kMagic;
  blk.version = kVersion;
  blk.motor = motor;
  blk.tracker = tracker;
  blk.mission = mission;
  blk.checksum = computeChecksum(blk);
  EEPROM.put(kEepromAddr, blk);
}

void clearSavedParams() {
  uint32_t zeroMagic = 0;
  EEPROM.put(kEepromAddr, zeroMagic);
}

}  // namespace Storage
