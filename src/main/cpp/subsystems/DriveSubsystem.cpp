// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "subsystems/DriveSubsystem.hpp"

#include <array>

#include "constants/Constants.h"
#include "frc/RobotBase.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StructArrayTopic.h"
#include "units/length.h"
#include "utils/TurboPoseEstimator.hpp"

DriveSubsystem::DriveSubsystem()
    : fleft(DriveSubsystemConstants::kFrontLeftDriveID, DriveSubsystemConstants::kFrontLeftSteerID,
            DriveSubsystemConstants::kFrontLeftEncoderID, DriveSubsystemConstants::kFrontLeftOffset,
            DriveSubsystemConstants::kCanivoreName),
      fright(DriveSubsystemConstants::kFrontRightDriveID, DriveSubsystemConstants::kFrontRightSteerID,
             DriveSubsystemConstants::kFrontRightEncoderID, DriveSubsystemConstants::kFrontRightOffset,
             DriveSubsystemConstants::kCanivoreName),
      bleft(DriveSubsystemConstants::kBackLeftDriveID, DriveSubsystemConstants::kBackLeftSteerID,
            DriveSubsystemConstants::kBackLeftEncoderID, DriveSubsystemConstants::kBackLeftOffset,
            DriveSubsystemConstants::kCanivoreName),
      bright(DriveSubsystemConstants::kBackRightDriveID, DriveSubsystemConstants::kBackRightSteerID,
             DriveSubsystemConstants::kBackRightEncoderID, DriveSubsystemConstants::kBackRightOffset,
             DriveSubsystemConstants::kCanivoreName),
      estimator(GetAngle(), GetModulePositions(), frc::Pose2d()) {
  m_posePublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Pose").Publish();
  m_speedsPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::ChassisSpeeds>("Speeds").Publish();
  m_swerveStatesPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("SwerveStates").Publish();

  m_posePublisher.Set(frc::Pose2d());
  m_speedsPublisher.Set(frc::ChassisSpeeds());
  m_swerveStatesPublisher.Set(GetModuleStates());

  m_lastTime = frc::Timer::GetFPGATimestamp();

  SetName("DriveSubsystem");
}

void DriveSubsystem::Drive(frc::ChassisSpeeds speeds) {
  m_cmdSpeeds = speeds;
  auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
  SetModuleStates(states);

  m_swerveStatesPublisher.Set(states);
  m_speedsPublisher.Set(speeds);
}

void DriveSubsystem::AutoDrive(frc::ChassisSpeeds speeds) {
  m_cmdSpeeds = speeds;

  speeds.vx *= -1;
  speeds.vy *= -1;
  speeds.omega *= -1;

  auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
  SetModuleStates(states);

  // undo negation to publish correct speeds
  auto normalStates = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(m_cmdSpeeds);
  m_swerveStatesPublisher.Set(normalStates);
  m_speedsPublisher.Set(m_cmdSpeeds);
}

void DriveSubsystem::SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states) {
  fleft.SetModuleState(states[0]);
  fright.SetModuleState(states[1]);
  bleft.SetModuleState(states[2]);
  bright.SetModuleState(states[3]);
}

void DriveSubsystem::DriverGryoZero() {
  driverGyroOffset = GetAngle();
}

std::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates() {
  return {fleft.GetModuleState(), fright.GetModuleState(), bleft.GetModuleState(), bright.GetModuleState()};
}

std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() {
  return {fleft.GetModulePosition(), fright.GetModulePosition(), bleft.GetModulePosition(), bright.GetModulePosition()};
}

frc::Rotation2d DriveSubsystem::GetDriverGyroAngle() {
  return GetAngle() - driverGyroOffset;
}

void DriveSubsystem::Periodic() {
  estimator.UpdateWithOdometryAndVision(GetAngle(), GetModulePositions());
  m_posePublisher.Set(estimator.getPose2D());

  frc::SmartDashboard::PutNumber("Gryo", GetAngle().Degrees().value());
}

void DriveSubsystem::SimulationPeriodic() {
  const auto currentTime = frc::Timer::GetFPGATimestamp();
  auto dt = currentTime - m_lastTime;
  m_lastTime = currentTime;

  auto fieldRelSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(m_cmdSpeeds, m_simPose.Rotation());

  auto newX = m_simPose.X() + fieldRelSpeeds.vx * dt;
  auto newY = m_simPose.Y() + fieldRelSpeeds.vy * dt;
  auto newTheta = frc::Rotation2d(m_simPose.Rotation().Radians() + fieldRelSpeeds.omega * dt);
  m_simPose = frc::Pose2d(newX, newY, newTheta);

  m_posePublisher.Set(m_simPose);

  auto& simGyro = pigeon.GetSimState();
  simGyro.SetRawYaw(m_simPose.Rotation().Degrees());
}
