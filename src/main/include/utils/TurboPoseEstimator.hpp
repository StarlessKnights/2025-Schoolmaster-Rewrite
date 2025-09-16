#pragma once

#include "constants/Constants.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"

class TurboPoseEstimator {
 private:
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

 public:
  TurboPoseEstimator(frc::Rotation2d gyroAngle,
                     std::array<frc::SwerveModulePosition, 4> modulePositions,
                     frc::Pose2d initialPose)
      : poseEstimator(DriveSubsystemConstants::kKinematics, gyroAngle, modulePositions,
                      initialPose) {}

  frc::Pose2d getPose2D();
  void resetEstimatorPosition(frc::Rotation2d gyroAngle,
                              std::array<frc::SwerveModulePosition, 4> modulePositions,
                              frc::Pose2d pose);
  void updateWithOdometry(frc::Rotation2d gyroAngle,
                          std::array<frc::SwerveModulePosition, 4> modulePositions);
};