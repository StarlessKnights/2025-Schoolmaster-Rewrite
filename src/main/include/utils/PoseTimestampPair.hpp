// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "frc/geometry/Pose2d.h"
#include "units/time.h"

class PoseTimestampPair {
 private:
  frc::Pose2d pose;
  units::second_t latency;

 public:
  PoseTimestampPair(frc::Pose2d pose, units::second_t latency) : pose(pose), latency(latency){};
  frc::Pose2d getPose() { return pose; };
  units::second_t getLatency() { return latency; };
};
