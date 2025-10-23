// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <functional>

#include "frc/geometry/Pose2d.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/DriveSubsystem.hpp"

class FollowPrecisePathCommand final : public frc2::CommandHelper<frc2::Command, FollowPrecisePathCommand> {
 private:
  DriveSubsystem* drive;
  std::function<frc::Pose2d()> goalSupplier;
  frc::Pose2d goalPose;

  frc::PIDController kXPrecisePathPID{3, 0, 0};
  frc::PIDController kYPrecisePathPID{3, 0, 0};
  frc::PIDController kRotatePrecisePathPID{2.0, 0, 0};

  nt::StructPublisher<frc::Pose2d> m_goalPosePublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("GoalPose").Publish();

 public:
  FollowPrecisePathCommand(DriveSubsystem* drive, std::function<frc::Pose2d()> goalSupplier);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
};
