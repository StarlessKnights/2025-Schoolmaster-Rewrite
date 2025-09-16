#include "subsystems/DriveSubsystem.hpp"

#include <array>

#include "constants/Constants.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"

DriveSubsystem::DriveSubsystem()
    : fleft(1, 2, 3, 0.0, DriveSubsystemConstants::kCanivoreName),
      fright(4, 5, 6, 0.0, DriveSubsystemConstants::kCanivoreName),
      bleft(7, 8, 9, 0.0, DriveSubsystemConstants::kCanivoreName),
      bright(11, 12, 13, 0.0, DriveSubsystemConstants::kCanivoreName) {}

void DriveSubsystem::drive(frc::ChassisSpeeds speeds) {
  auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
  setModuleStates(states);
}

void DriveSubsystem::setModuleStates(const std::array<frc::SwerveModuleState, 4> &states) {
  fleft.setModuleState(states[0]);
  fright.setModuleState(states[1]);
  bleft.setModuleState(states[2]);
  bright.setModuleState(states[3]);
}

std::array<frc::SwerveModuleState, 4> DriveSubsystem::getModuleStates() {
  return {fleft.getModuleState(), fright.getModuleState(), bleft.getModuleState(),
          bright.getModuleState()};
}

std::array<frc::SwerveModulePosition, 4> DriveSubsystem::getModulePositions() {
  return {fleft.getModulePosition(), fright.getModulePosition(), bleft.getModulePosition(),
          bright.getModulePosition()};
}

frc::Rotation2d DriveSubsystem::getDriverGyroAngle() { return getAngle() - driverGyroOffset; }