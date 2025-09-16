#include <frc/kinematics/ChassisSpeeds.h>

#include <commands/FieldDriveCommand.hpp>

#include "frc/smartdashboard/SmartDashboard.h"
#include "units/angular_velocity.h"

void FieldDriveCommand::Initialize() {};

void FieldDriveCommand::Execute() {
  frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      ySpeed() * 4.5_mps, xSpeed() * 4.5_mps, rotX() * units::radians_per_second_t{570},
      drive->getDriverGyroAngle());

  drive->drive(fieldRelativeSpeeds);
  frc::SmartDashboard::PutNumber("Field Relative Speed X", fieldRelativeSpeeds.vx.value());
  frc::SmartDashboard::PutNumber("Field Relative Speed Y", fieldRelativeSpeeds.vy.value());
  frc::SmartDashboard::PutNumber("Field Relative Rot", fieldRelativeSpeeds.omega.value());
}

void FieldDriveCommand::End(bool interrupted) {};

bool FieldDriveCommand::IsFinished() { return false; }