// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "commands/FieldDriveCommand.hpp"

#include <frc/kinematics/ChassisSpeeds.h>

#include "constants/Constants.h"

void FieldDriveCommand::Initialize() {};

void FieldDriveCommand::Execute() {
  frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      ySpeed() * DriveSubsystemConstants::kMaxLinearSpeed, xSpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
      rotX() * DriveSubsystemConstants::kMaxAngularSpeed, drive->GetDriverGyroAngle());

  drive->Drive(fieldRelativeSpeeds);
}

void FieldDriveCommand::End(bool) {};

bool FieldDriveCommand::IsFinished() {
  return false;
}
