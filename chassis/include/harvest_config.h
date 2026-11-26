#pragma once

#include "harvest_run.h"

// 单边停车版使用的一台相机。2=D17，3=D15，以实际接线为准。
constexpr uint8_t HARVEST_CAMERA_PORT = 2;
// 新机构由同一舵机选高低角。台架标定后填写三个绝对角度（0..180），-1 禁用出杆。
constexpr uint8_t HARVEST_WORK_SERVO_CHANNEL = 3;  // 左上舵机实际接 D35，同一路处理高低果
constexpr int16_t HARVEST_WORK_HOME_DEG = -1;
constexpr int16_t HARVEST_WORK_HIGH_DEG = 80;
constexpr int16_t HARVEST_WORK_LOW_DEG = 105;
constexpr bool HARVEST_WORK_READY = HARVEST_WORK_HOME_DEG >= 0 && HARVEST_WORK_HOME_DEG <= 180 &&
    HARVEST_WORK_HIGH_DEG >= 0 && HARVEST_WORK_HIGH_DEG <= 180 &&
    HARVEST_WORK_LOW_DEG >= 0 && HARVEST_WORK_LOW_DEG <= 180 &&
    HARVEST_WORK_HOME_DEG != HARVEST_WORK_HIGH_DEG &&
    HARVEST_WORK_HOME_DEG != HARVEST_WORK_LOW_DEG && HARVEST_WORK_HIGH_DEG != HARVEST_WORK_LOW_DEG;
// 单边单沟工位表。按行车方向填写 {开始, 最迟减速, 结束}，从发车点量，单位 mm。
// 窗口须按顺序且互不重叠。未测时保留无效占位，仍可测试循迹和减速。
constexpr HarvestStation HARVEST_LANE_STATIONS[] = {{-1, -1, -1}};
constexpr size_t HARVEST_LANE_STATION_COUNT = sizeof(HARVEST_LANE_STATIONS) / sizeof(HARVEST_LANE_STATIONS[0]);
constexpr bool harvestLaneStationsValid(size_t index = 0) {
  return index == HARVEST_LANE_STATION_COUNT ||
      (HARVEST_LANE_STATIONS[index].valid() &&
       (index == 0 || HARVEST_LANE_STATIONS[index - 1].endMm <= HARVEST_LANE_STATIONS[index].startMm) &&
       harvestLaneStationsValid(index + 1));
}
static_assert(harvestLaneStationsValid() || (HARVEST_LANE_STATION_COUNT == 1 &&
              HARVEST_LANE_STATIONS[0].startMm == -1 && HARVEST_LANE_STATIONS[0].slowByMm == -1 &&
              HARVEST_LANE_STATIONS[0].endMm == -1), "Invalid or overlapping harvest stations");
// 比赛单工位接入。0 禁用；填写任务内沟序号（第一条实际为 7 号沟）及从入沟起量的三点。
constexpr uint8_t HARVEST_MISSION_LANE = 0;
constexpr HarvestStation HARVEST_STATION{-1, -1, -1};
constexpr int16_t HARVEST_SCAN_PWM = 20;
// 以下均为首轮试验初值。以装车录像、轮速及黄果净空调整。
constexpr uint32_t HARVEST_FRAME_FRESH_MS = 300;
constexpr uint32_t HARVEST_SCAN_MAX_MS = 8000;
constexpr uint32_t HARVEST_STRIKE_CYCLE_MS = 1000;
constexpr uint8_t HARVEST_NEAR_FRAMES = 3;
constexpr float HARVEST_STOPPED_WHEEL_MMS = 5.0f;
constexpr uint32_t HARVEST_SETTLE_MIN_MS = 250;
constexpr uint32_t HARVEST_SETTLE_MAX_MS = 1300;
constexpr uint8_t HARVEST_STOPPED_ALIGNED_FRAMES = 2;
constexpr uint32_t HARVEST_TREE_HOLD_MS = 500;
constexpr float HARVEST_MIN_MOVING_MMS = 10.0f;
constexpr float HARVEST_MAX_STRIKE_MMS = 120.0f;

// 双边单沟测试初值。巡航档低于比赛档，留出相机确认和减速距离。
constexpr int16_t HARVEST_TEST_CRUISE_PWM = 45;
constexpr uint8_t HARVEST_CLEAR_FRAMES = 5;
constexpr float HARVEST_CLEAR_TRAVEL_MM = 45.0f;
constexpr uint32_t HARVEST_START_GAP_MS = 100;
constexpr uint16_t HARVEST_END_BRAKE_MS = 220;
constexpr uint16_t HARVEST_END_RESERVE_MM = 20;
constexpr uint32_t HARVEST_TEST_MAX_MS = 60000;
// 只作故障上界，走到端头时仍用前超声停车。场地沟长约 3000 mm。
constexpr float HARVEST_TEST_MAX_MM = 3300.0f;
constexpr bool HARVEST_LEFT_IS_PORT2 = true;
