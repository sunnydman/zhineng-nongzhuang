// ============================================================================
// 【文件作用】EEPROM 掉电参数保存头文件（声明接口，平时调参不用改）
//   - 用于把串口在线调好的参数永久存入 Mega 2560 内置 EEPROM（SAVE / LOAD）。
// ============================================================================
#pragma once

#include <Arduino.h>
#include "mission.h"
#include "motor_driver.h"
#include "tracker.h"

namespace Storage {

bool loadParams(MotorParams& motor, TrackerParams& tracker, MissionParams& mission);
void saveParams(const MotorParams& motor, const TrackerParams& tracker, const MissionParams& mission);
void clearSavedParams();

}  // namespace Storage
