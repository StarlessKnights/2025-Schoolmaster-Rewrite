#include "utils/TurboPoseEstimator.hpp"

frc::Pose2d TurboPoseEstimator::getPose2D() { return poseEstimator.GetEstimatedPosition(); }

void TurboPoseEstimator::resetEstimatorPosition(
    frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions,
    frc::Pose2d pose) {
  poseEstimator.ResetPosition(gyroAngle, modulePositions, pose);
}

void TurboPoseEstimator::updateWithOdometry(
    frc::Rotation2d gyroAngle, std::array<frc::SwerveModulePosition, 4> modulePositions) {
  poseEstimator.Update(gyroAngle, modulePositions);
}