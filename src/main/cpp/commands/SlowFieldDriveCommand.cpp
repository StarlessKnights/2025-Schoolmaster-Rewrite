// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "commands/SlowFieldDriveCommand.hpp"

#include <frc/kinematics/ChassisSpeeds.h>

#include "units/angular_velocity.h"

void SlowFieldDriveCommand::Initialize() {};

void SlowFieldDriveCommand::Execute() {
  frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      ySpeed() * 0.5_mps, xSpeed() * 0.5_mps, rotX() * units::radians_per_second_t{180_deg_per_s},
      drive->GetDriverGyroAngle());

  drive->Drive(fieldRelativeSpeeds);
}

void SlowFieldDriveCommand::End(bool interrupted) {};

bool SlowFieldDriveCommand::IsFinished() {
  return false;
}
