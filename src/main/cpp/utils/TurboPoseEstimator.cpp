#include "utils/TurboPoseEstimator.hpp"

#include "frc/geometry/Pose2d.h"
#include "utils/PoseTimestampPair.hpp"
#include <optional>

frc::Pose2d TurboPoseEstimator::getPose2D() {
  frc::Pose2d pose = poseEstimator.GetEstimatedPosition();

  return pose;
}

void TurboPoseEstimator::ResetEstimatorPosition(frc::Rotation2d gyroAngle,
                                                std::array<frc::SwerveModulePosition, 4> modulePositions,
                                                frc::Pose2d pose) {
  poseEstimator.ResetPosition(gyroAngle, modulePositions, pose);
}

void TurboPoseEstimator::UpdateWithOdometryAndVision(frc::Rotation2d gyroAngle,
                                                     std::array<frc::SwerveModulePosition, 4> modulePositions) {
  UpdateWithAllAvailableVisionMeasurements();
  poseEstimator.Update(gyroAngle, modulePositions);
}

void TurboPoseEstimator::TryVisionUpdateWithCamera(TurboPhotonCamera &camera) {
  std::optional<PoseTimestampPair> visionPose = camera.fetchPose();

  if (visionPose.has_value()) {
    poseEstimator.AddVisionMeasurement(visionPose->getPose(), visionPose->getLatency());
  }
}

void TurboPoseEstimator::UpdateWithAllAvailableVisionMeasurements() {
  for (auto &camera : localizationCameras) {
    TryVisionUpdateWithCamera(camera);
  }
}
