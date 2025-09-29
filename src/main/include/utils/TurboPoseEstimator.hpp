#pragma once

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "global/Globals.h"

class TurboPoseEstimator {
private:
  frc::SwerveDrivePoseEstimator<4> poseEstimator;

public:
  TurboPoseEstimator(frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions,
                     frc::Pose2d initialPose)
      : poseEstimator(DriveSubsystemConstants::swerveKinematics, gyroAngle, modulePositions, initialPose) {}

  frc::Pose2d getPose2D();
  void ResetEstimatorPosition(frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions,
                              frc::Pose2d pose);
  void UpdateWithOdometry(frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions);
};
