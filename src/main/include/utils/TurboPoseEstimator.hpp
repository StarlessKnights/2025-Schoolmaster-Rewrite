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
  bool seesTag;

  std::array<TurboPhotonCamera, 2> localizationCameras = {
      TurboPhotonCamera(CameraConstants::kLocalizationCamOneName, CameraConstants::kLocalizationCamOneOffset),
      TurboPhotonCamera(CameraConstants::kLocalizationCamTwoName, CameraConstants::kLocalizationCamTwoOffset)};

  nt::StructSubscriber<frc::Pose2d> simPoseTopic =
      nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Pose").Subscribe(frc::Pose2d{});

 public:
  TurboPoseEstimator(const frc::Rotation2d& gyroAngle, const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                     const frc::Pose2d& initialPose)
      : poseEstimator(DriveSubsystemConstants::kKinematics, gyroAngle, modulePositions, initialPose), seesTag(false) {}

  frc::Pose2d GetPose2D() const;
  void ResetEstimatorPosition(const frc::Rotation2d& gyroAngle,
                              const std::array<frc::SwerveModulePosition, 4>& modulePositions, const frc::Pose2d& pose);
  void UpdateWithOdometryAndVision(const frc::Rotation2d& gyroAngle,
                                   const std::array<frc::SwerveModulePosition, 4>& modulePositions);
  void TryVisionUpdateWithCamera(TurboPhotonCamera& camera);
  void UpdateWithAllAvailableVisionMeasurements();

  bool SeesTag() const { return seesTag; }
};
