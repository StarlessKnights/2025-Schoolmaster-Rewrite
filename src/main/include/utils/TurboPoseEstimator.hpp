#pragma once

#include "constants/Constants.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "utils/TurboPhotonCamera.hpp"
#include <array>

class TurboPoseEstimator {
private:
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

  std::array<class TurboPhotonCamera, 2> localizationCameras = {
      TurboPhotonCamera(CameraConstants::kLocalizationCamOneName, CameraConstants::kLocalizationCamOneOffset),
      TurboPhotonCamera(CameraConstants::kLocalizationCamTwoName, CameraConstants::kLocalizationCamTwoOffset)};

public:
  TurboPoseEstimator(frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions,
                     frc::Pose2d initialPose)
      : poseEstimator(DriveSubsystemConstants::kKinematics, gyroAngle, modulePositions, initialPose) {}

  frc::Pose2d getPose2D();
  void ResetEstimatorPosition(frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions,
                              frc::Pose2d pose);
  void UpdateWithOdometry(frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions);
  void TryVisionUpdateWithCamera(const TurboPhotonCamera &camera);
};