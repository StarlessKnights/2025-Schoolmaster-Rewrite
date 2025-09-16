#pragma once

#include "constants/Constants.h"
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "subsystems/DriveSubsystem.hpp"

class TurboPoseEstimator {
 private:
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

 public:
  TurboPoseEstimator(DriveSubsystem &drive, frc::Pose2d initialPose)
      : poseEstimator(DriveSubsystemConstants::kKinematics, drive.getAngle(),
                      drive.getModulePositions(), initialPose) {}

  frc::Pose2d getPose2D();
  void resetEstimatorPosition(frc::Rotation2d gyroAngle,
                              std::array<frc::SwerveModulePosition, 4> modulePositions,
                              frc::Pose2d pose);
  void updateWithOdometry(frc::Rotation2d gyroAngle,
                          std::array<frc::SwerveModulePosition, 4> modulePositions);
};