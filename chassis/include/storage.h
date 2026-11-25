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
