// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "utils/TurboPoseEstimator.hpp"

#include "frc/RobotBase.h"
#include "frc/geometry/Pose2d.h"
#include "utils/PoseTimestampPair.hpp"

frc::Pose2d TurboPoseEstimator::GetPose2D() const {
  return poseEstimator.GetEstimatedPosition();
}

void TurboPoseEstimator::ResetEstimatorPosition(const frc::Rotation2d& gyroAngle,
                                                const std::array<frc::SwerveModulePosition, 4>& modulePositions,
                                                const frc::Pose2d& pose) {
  poseEstimator.ResetPosition(gyroAngle, modulePositions, pose);
}

void TurboPoseEstimator::UpdateWithOdometryAndVision(const frc::Rotation2d& gyroAngle,
                                                     const std::array<frc::SwerveModulePosition, 4>& modulePositions) {
  poseEstimator.Update(gyroAngle, modulePositions);
  UpdateWithAllAvailableVisionMeasurements();
}

void TurboPoseEstimator::TryVisionUpdateWithCamera(TurboPhotonCamera& camera) {
  if constexpr (frc::RobotBase::IsSimulation()) {
    const frc::Pose2d pose = simPoseTopic.Get(frc::Pose2d());
    camera.UpdateSim(pose);
  }

  const std::vector<PoseTimestampPair> visionPoses = camera.FetchPose();

  for (const auto& pair : visionPoses) {
    poseEstimator.AddVisionMeasurement(pair.getPose(), pair.getLatency());
  }
}

void TurboPoseEstimator::UpdateWithAllAvailableVisionMeasurements() {
  for (auto& camera : localizationCameras) {
    TryVisionUpdateWithCamera(camera);
  }
}

bool TurboPoseEstimator::SeesTag() const {
  bool seesTag = false;

  for (const auto& camera : localizationCameras) {
    if (camera.SeesTag()) {
      seesTag = true;
      break;
    }
  }

  return seesTag;
}
