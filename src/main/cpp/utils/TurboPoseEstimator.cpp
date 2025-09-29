#include "utils/TurboPoseEstimator.hpp"

#include "frc/RobotBase.h"
#include "frc/geometry/Pose2d.h"

#include "networktables/NetworkTableInstance.h"
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
  if (frc::RobotBase::IsSimulation()) {
    auto pose =
        nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Pose").Subscribe(frc::Pose2d{}).Get();

    camera.updateSim(pose);
  }

  std::optional<PoseTimestampPair> visionPose = camera.fetchPose();

  if (visionPose.has_value()) {
    posePublisher.Set(visionPose->getPose());
    poseEstimator.AddVisionMeasurement(visionPose->getPose(), visionPose->getLatency());
  }
}

void TurboPoseEstimator::UpdateWithAllAvailableVisionMeasurements() {
  for (auto &camera : localizationCameras) {
    TryVisionUpdateWithCamera(camera);
  }
}
