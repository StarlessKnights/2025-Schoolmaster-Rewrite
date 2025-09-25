#pragma once

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/geometry/Transform3d.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "utils/PoseTimestampPair.hpp"
#include <optional>

class TurboPhotonCamera {
private:
  frc::AprilTagFieldLayout layout;
  photon::PhotonCamera camera;
  photon::PhotonPoseEstimator poseEstimator;

public:
  TurboPhotonCamera(const std::string_view cameraName, frc::Transform3d cameraInBotSpace);
  frc::AprilTagFieldLayout getLayout();
  photon::PhotonPipelineResult getLatestResult();
  std::optional<photon::EstimatedRobotPose> getCameraEstimatedPose3d();
  class PoseTimestampPair fetchPose();
};