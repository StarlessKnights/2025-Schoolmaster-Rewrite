// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "commands/autoalign/FollowPrecisePathCommand.hpp"

#include <algorithm>
#include <functional>

#include "frc/RobotBase.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "subsystems/DriveSubsystem.hpp"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"

FollowPrecisePathCommand::FollowPrecisePathCommand(DriveSubsystem* drive, std::function<frc::Pose2d()> goalSupplier)
    : drive(drive), goalSupplier(goalSupplier) {
  kXPrecisePathPID.SetTolerance(0.01);
  kYPrecisePathPID.SetTolerance(0.01);
  kRotatePrecisePathPID.SetTolerance(0.1);

  kRotatePrecisePathPID.EnableContinuousInput(0, std::numbers::pi * 2);

  SetName("FollowPrecisePathCommand");

  AddRequirements(drive);
}

void FollowPrecisePathCommand::Initialize() {
  goalPose = goalSupplier();

  kXPrecisePathPID.Reset();
  kYPrecisePathPID.Reset();
  kRotatePrecisePathPID.Reset();

  m_goalPosePublisher.Set(goalPose);
}

void FollowPrecisePathCommand::Execute() {
  frc::Pose2d currentPose;

  if (frc::RobotBase::IsReal()) {
    currentPose = drive->GetPose();
  } else {
    currentPose = drive->GetSimPose();
  }

  double xSpeed = kXPrecisePathPID.Calculate(currentPose.X().to<double>(), goalPose.X().to<double>());
  double ySpeed = kYPrecisePathPID.Calculate(currentPose.Y().to<double>(), goalPose.Y().to<double>());
  double rotSpeed = kRotatePrecisePathPID.Calculate(currentPose.Rotation().Radians().to<double>(),
                                                    goalPose.Rotation().Radians().to<double>());

  xSpeed = std::clamp(xSpeed, -0.5, 0.5);
  ySpeed = std::clamp(ySpeed, -0.5, 0.5);

  drive->Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      units::meters_per_second_t{-xSpeed}, units::meters_per_second_t{-ySpeed}, units::radians_per_second_t{-rotSpeed},
      currentPose.Rotation()));
}

void FollowPrecisePathCommand::End(bool) {
  drive->Drive(frc::ChassisSpeeds{0.0_mps, 0.0_mps, 0.0_rad_per_s});
}

bool FollowPrecisePathCommand::IsFinished() {
  return kXPrecisePathPID.AtSetpoint() && kYPrecisePathPID.AtSetpoint() && kRotatePrecisePathPID.AtSetpoint();
}
