#pragma once

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform3d.h"
#include "networktables/StructArrayTopic.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/simulation/PhotonCameraSim.h"
#include "photon/simulation/VisionSystemSim.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "utils/PoseTimestampPair.hpp"
#include <optional>

class TurboPhotonCamera {
private:
  frc::AprilTagFieldLayout layout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);
  photon::PhotonCamera camera;
  photon::PhotonPoseEstimator poseEstimator;

  std::optional<photon::VisionSystemSim> systemSim = std::nullopt;
  std::optional<photon::PhotonCameraSim> cameraSim = std::nullopt;

  nt::StructArrayPublisher<frc::Pose2d> visionTargetPublisher;

public:
  TurboPhotonCamera(const std::string &cameraName, frc::Transform3d cameraInBotSpace);
  void updateSim(frc::Pose2d robotPose);
  photon::PhotonPipelineResult getLatestResult();
  std::optional<photon::EstimatedRobotPose> getCameraEstimatedPose3d();
  std::optional<PoseTimestampPair> fetchPose();
  int getNumTargets();
};
