// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "subsystems/DriveSubsystem.hpp"

#include <array>

#include "constants/Constants.h"
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

#include <frc/DataLogManager.h>
#include <frc2/command/FunctionalCommand.h>

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

void DriveSubsystem::Drive(const frc::ChassisSpeeds& speeds) {
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

  const auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
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

void DriveSubsystem::DriverGyroZero() {
  driverGyroOffset = GetAngle();
}

std::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates() {
  return {fleft.GetModuleState(), fright.GetModuleState(), bleft.GetModuleState(), bright.GetModuleState()};
}

std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() {
  return {fleft.GetModulePosition(), fright.GetModulePosition(), bleft.GetModulePosition(), bright.GetModulePosition()};
}

frc::Rotation2d DriveSubsystem::GetDriverGyroAngle() const {
  return GetAngle() - driverGyroOffset;
}

frc2::CommandPtr DriveSubsystem::DriveCommand(std::function<double()>&& xSpeed, std::function<double()>&& ySpeed,
                                              std::function<double()>&& rotSpeed) {
  return frc2::FunctionalCommand(
             [] {},
             [this, xSpeed = std::move(xSpeed), ySpeed = std::move(ySpeed), rotSpeed = std::move(rotSpeed)] {
               const frc::ChassisSpeeds fieldRelativeSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                   ySpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
                   xSpeed() * DriveSubsystemConstants::kMaxLinearSpeed,
                   rotSpeed() * DriveSubsystemConstants::kMaxAngularSpeed, GetDriverGyroAngle());

               Drive(fieldRelativeSpeeds);
             },
             [](bool) {}, [] { return false; }, {this})
      .ToPtr()
      .WithName("DriveCommand");
}

void DriveSubsystem::Periodic() {
  if constexpr (frc::RobotBase::IsReal()) {
    estimator.UpdateWithOdometryAndVision(GetAngle(), GetModulePositions());

    m_posePublisher.Set(estimator.GetPose2D());
  }

  frc::SmartDashboard::PutNumber("Gyro", GetAngle().Degrees().value());
}

void DriveSubsystem::SimulationPeriodic() {
  const auto currentTime = frc::Timer::GetFPGATimestamp();
  const auto dt = currentTime - m_lastTime;
  m_lastTime = currentTime;

  auto [vx, vy, omega] = frc::ChassisSpeeds::FromRobotRelativeSpeeds(m_cmdSpeeds, m_simPose.Rotation());

  const auto newX = m_simPose.X() + vx * dt;
  const auto newY = m_simPose.Y() + vy * dt;
  const auto newTheta = frc::Rotation2d(m_simPose.Rotation().Radians() + omega * dt);
  m_simPose = frc::Pose2d(newX, newY, newTheta);

  m_posePublisher.Set(m_simPose);

  auto& simGyro = pigeon.GetSimState();
  simGyro.SetRawYaw(m_simPose.Rotation().Degrees());
}
