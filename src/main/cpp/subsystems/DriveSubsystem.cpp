#include "subsystems/DriveSubsystem.hpp"

#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/kinematics/SwerveModuleState.h"

DriveSubsystem::DriveSubsystem()
    : fleft(1, 2, 3, 0.0, DriveSubsystemConstants::kCanivoreName),
      fright(4, 5, 6, 0.0, DriveSubsystemConstants::kCanivoreName),
      bleft(7, 8, 9, 0.0, DriveSubsystemConstants::kCanivoreName),
      bright(11, 12, 13, 0.0, DriveSubsystemConstants::kCanivoreName) {
  frc::DataLogManager::Log("DriveSubsystem initialized");
}

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

frc::Rotation2d DriveSubsystem::getDriverGyroAngle() { return getAngle() - driverGyroOffset; }