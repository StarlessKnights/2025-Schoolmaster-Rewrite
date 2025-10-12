// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <array>

#include "constants/Constants.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "networktables/StructTopic.h"
#include "utils/TurboPhotonCamera.hpp"

class TurboPoseEstimator {
 private:
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

  std::array<class TurboPhotonCamera, 2> localizationCameras = {
      TurboPhotonCamera(CameraConstants::kLocalizationCamOneName, CameraConstants::kLocalizationCamOneOffset),
      TurboPhotonCamera(CameraConstants::kLocalizationCamTwoName, CameraConstants::kLocalizationCamTwoOffset)};

  nt::StructPublisher<frc::Pose2d> posePublisher =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Vision Pose").Publish();
  nt::StructSubscriber<frc::Pose2d> simPoseTopic =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Pose").Subscribe(frc::Pose2d{});

 public:
  TurboPoseEstimator(frc::Rotation2d gyroAngle, const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                     const frc::Pose2d& initialPose)
      : poseEstimator(DriveSubsystemConstants::kKinematics, gyroAngle, modulePositions, initialPose) {}

  frc::Pose2d GetPose2D() const;
  void ResetEstimatorPosition(frc::Rotation2d gyroAngle,
                              const std::array<frc::SwerveModulePosition, 4>& modulePositions, const frc::Pose2d& pose);
  void UpdateWithOdometryAndVision(frc::Rotation2d gyroAngle,
                                   const std::array<frc::SwerveModulePosition, 4>& modulePositions);
  void TryVisionUpdateWithCamera(TurboPhotonCamera& camera);
  void UpdateWithAllAvailableVisionMeasurements();
};
