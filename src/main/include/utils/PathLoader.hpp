// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "subsystems/DriveSubsystem.hpp"
#include "utils/TurboPoseEstimator.hpp"

class PathLoader {
 public:
  static void ConfigurePathPlanner(DriveSubsystem& drive, TurboPoseEstimator& poseEstimator);
};
