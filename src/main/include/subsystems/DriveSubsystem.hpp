// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <array>

#include "constants/Constants.h"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "frc/RobotBase.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc2/command/SubsystemBase.h"
#include "networktables/StructArrayTopic.h"
#include "networktables/StructTopic.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "utils/NeoKrakenModule.hpp"
#include "utils/TurboPoseEstimator.hpp"

class DriveSubsystem : public frc2::SubsystemBase {
 private:
  NeoKrakenModule fleft, fright, bleft, bright;
  ctre::phoenix6::hardware::Pigeon2 pigeon{DriveSubsystemConstants::kPigeonID, DriveSubsystemConstants::kCanivoreName};
  frc::Rotation2d driverGyroOffset{};
  TurboPoseEstimator estimator;

  frc::ChassisSpeeds m_cmdSpeeds{0.0_mps, 0.0_mps, 0.0_rad_per_s};
  frc::Pose2d m_simPose{};
  units::second_t m_lastTime{0.0_s};

  nt::StructPublisher<frc::Pose2d> m_posePublisher;
  nt::StructPublisher<frc::ChassisSpeeds> m_speedsPublisher;
  nt::StructArrayPublisher<frc::SwerveModuleState> m_swerveStatesPublisher;

 public:
  DriveSubsystem();

  void Drive(frc::ChassisSpeeds speeds);
  void AutoDrive(frc::ChassisSpeeds speeds);
  void SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states);
  void DriverGryoZero();
  frc::Rotation2d GetDriverGyroAngle();
  frc::Rotation2d GetAngle() { return pigeon.GetRotation2d(); }
  std::array<frc::SwerveModuleState, 4> GetModuleStates();
  std::array<frc::SwerveModulePosition, 4> GetModulePositions();

  frc::Pose2d GetPose() { return estimator.getPose2D(); }
  frc::Pose2d GetSimPose() { return m_simPose; }

  frc::ChassisSpeeds GetRobotRelativeChassisSpeeds() {
    if (frc::RobotBase::IsReal()) {
      return DriveSubsystemConstants::kKinematics.ToChassisSpeeds(GetModuleStates());
    } else {
      return m_cmdSpeeds;
    }
  }

  TurboPoseEstimator& GetPoseEstimator() { return estimator; }

  void Periodic() override;
  void SimulationPeriodic() override;
};
