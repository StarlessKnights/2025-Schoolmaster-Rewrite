#include <frc/kinematics/ChassisSpeeds.h>

#include <commands/FieldDriveCommand.hpp>

#include "constants/Constants.h"

void FieldDriveCommand::Initialize() {};

void FieldDriveCommand::Execute() {
  frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      ySpeed() * DriveSubsystemConstants::kMaxLinearSpeed, xSpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
      rotX() * DriveSubsystemConstants::kMaxAngularSpeed, drive->GetDriverGyroAngle());

  drive->Drive(fieldRelativeSpeeds);
}

void FieldDriveCommand::End(bool interrupted) {};

bool FieldDriveCommand::IsFinished() { return false; }
