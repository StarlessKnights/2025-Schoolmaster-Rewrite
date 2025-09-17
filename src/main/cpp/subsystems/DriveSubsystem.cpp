#include "subsystems/DriveSubsystem.hpp"

#include <array>

#include "constants/Constants.h"
#include "frc/Timer.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "utils/TurboPoseEstimator.hpp"

DriveSubsystem::DriveSubsystem()
    : fleft(1, 2, 3, 0.0, DriveSubsystemConstants::kCanivoreName),
      fright(4, 5, 6, 0.0, DriveSubsystemConstants::kCanivoreName),
      bleft(7, 8, 9, 0.0, DriveSubsystemConstants::kCanivoreName),
      bright(11, 12, 13, 0.0, DriveSubsystemConstants::kCanivoreName),
      field(),
      estimator(getAngle(), getModulePositions(), frc::Pose2d()) {
  frc::SmartDashboard::PutData("Field", &field);
  m_lastTime = frc::Timer::GetFPGATimestamp();
}

void DriveSubsystem::drive(frc::ChassisSpeeds speeds) {
  m_cmdSpeeds = speeds;
  auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
  setModuleStates(states);
}

void DriveSubsystem::setModuleStates(const std::array<frc::SwerveModuleState, 4>& states) {
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

void DriveSubsystem::Periodic() {
  estimator.updateWithOdometry(getAngle(), getModulePositions());
  field.SetRobotPose(estimator.getPose2D());

  frc::SmartDashboard::PutNumber("Front Left", fleft.getEncoderPosition());
  frc::SmartDashboard::PutNumber("Front Right", fright.getEncoderPosition());
  frc::SmartDashboard::PutNumber("Back Left", bleft.getEncoderPosition());
  frc::SmartDashboard::PutNumber("Back Right", bright.getEncoderPosition());
}

void DriveSubsystem::SimulationPeriodic() {
  const auto currentTime = frc::Timer::GetFPGATimestamp();
  auto dt = currentTime - m_lastTime;
  m_lastTime = currentTime;

  auto fieldRelSpeeds =
      frc::ChassisSpeeds::FromRobotRelativeSpeeds(m_cmdSpeeds, m_simPose.Rotation());

  auto newX = m_simPose.X() + fieldRelSpeeds.vx * dt;
  auto newY = m_simPose.Y() + fieldRelSpeeds.vy * dt;
  auto newTheta = frc::Rotation2d(m_simPose.Rotation().Radians() + fieldRelSpeeds.omega * dt);
  m_simPose = frc::Pose2d(newX, newY, newTheta);

  field.SetRobotPose(m_simPose);

  auto& simGyro = pigeon.GetSimState();
  simGyro.SetRawYaw(m_simPose.Rotation().Degrees());
}