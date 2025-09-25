#include "utils/TurboPhotonCamera.hpp"
#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Transform3d.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "utils/PoseTimestampPair.hpp"
#include <exception>
#include <optional>
#include <string_view>
#include <vector>

TurboPhotonCamera::TurboPhotonCamera(const std::string_view cameraName, frc::Transform3d cameraInBotSpace)
    : layout(getLayout()), camera(cameraName),
      poseEstimator(layout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, cameraInBotSpace) {}

frc::AprilTagFieldLayout TurboPhotonCamera::getLayout() {
  try {
    layout = frc::AprilTagFieldLayout{CameraConstants::kPathToAprilTagLayout};
    frc::DataLogManager::Log("Successfully loaded edited json (April tag field layout)");
  } catch (std::exception &e) {
    frc::DataLogManager::Log("Failed to load edited json (April tag field layout): " + std::string(e.what()));
    layout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);
  }

  return layout;
}

photon::PhotonPipelineResult TurboPhotonCamera::getLatestResult() {
  std::vector<photon::PhotonPipelineResult> results = camera.GetAllUnreadResults();

  if (results.empty()) {
    return photon::PhotonPipelineResult{};
  } else {
    return results.back();
  }
}

std::optional<photon::EstimatedRobotPose> TurboPhotonCamera::getCameraEstimatedPose3d() {
  auto result = getLatestResult();
  return poseEstimator.Update(result);
}

class PoseTimestampPair TurboPhotonCamera::fetchPose() {
  std::optional<photon::EstimatedRobotPose> ret;

  try {
    ret = getCameraEstimatedPose3d();
  } catch (const std::exception &) {
    ret = std::nullopt;
  }

  if (ret.has_value() && getLatestResult().GetTargets().size() >= 1) {
    return PoseTimestampPair{ret->estimatedPose.ToPose2d(), ret->timestamp};
  }

  return PoseTimestampPair{frc::Pose2d(), -1_s};
}