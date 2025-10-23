// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "frc/geometry/Pose2d.h"
#include "units/time.h"

class PoseTimestampPair {
  frc::Pose2d pose;
  units::second_t latency;

 public:
  PoseTimestampPair(const frc::Pose2d& pose, const units::second_t latency) : pose(pose), latency(latency) {}
  [[nodiscard]] frc::Pose2d getPose() const { return pose; }
  [[nodiscard]] units::second_t getLatency() const { return latency; }
};
