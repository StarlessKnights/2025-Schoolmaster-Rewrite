// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "utils/TurboPoseEstimator.hpp"

#include <optional>

#include "frc/RobotBase.h"
#include "frc/geometry/Pose2d.h"
#include "utils/PoseTimestampPair.hpp"

frc::Pose2d TurboPoseEstimator::GetPose2D() const {
  return poseEstimator.GetEstimatedPosition();
}

void TurboPoseEstimator::ResetEstimatorPosition(frc::Rotation2d gyroAngle,
                                                const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                                                const frc::Pose2d& pose) {
  poseEstimator.ResetPosition(gyroAngle, modulePositions, pose);
}

void TurboPoseEstimator::UpdateWithOdometryAndVision(frc::Rotation2d gyroAngle,
                                                     const std::array<frc::SwerveModulePosition, 4>& modulePositions) {
  poseEstimator.Update(gyroAngle, modulePositions);
  UpdateWithAllAvailableVisionMeasurements();

  posePublisher.Set(poseEstimator.GetEstimatedPosition());
}

void TurboPoseEstimator::TryVisionUpdateWithCamera(TurboPhotonCamera& camera) {
  if (frc::RobotBase::IsSimulation()) {
    frc::Pose2d pose = simPoseTopic.Get(frc::Pose2d());
    camera.UpdateSim(pose);
  }

  std::optional<PoseTimestampPair> visionPose = camera.FetchPose();

  if (!visionPose.has_value()) {
    return;
  }

  posePublisher.Set(visionPose->getPose());
  poseEstimator.AddVisionMeasurement(visionPose->getPose(), visionPose->getLatency());
}

void TurboPoseEstimator::UpdateWithAllAvailableVisionMeasurements() {
  for (auto& camera : localizationCameras) {
    TryVisionUpdateWithCamera(camera);
  }
}
