#pragma once

#include "frc/geometry/Pose2d.h"
#include "units/time.h"

class PoseTimestampPair {
private:
  frc::Pose2d pose;
  units::second_t latency;

public:
  PoseTimestampPair(frc::Pose2d pose, units::second_t latency) : pose(pose), latency(latency) {};
};