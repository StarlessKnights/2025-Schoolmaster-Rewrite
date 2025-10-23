// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "utils/TurboPhotonCamera.hpp"

#include <exception>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/simulation/PhotonCameraSim.h"
#include "photon/simulation/SimCameraProperties.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "units/frequency.h"
#include "units/time.h"
#include "utils/PoseTimestampPair.hpp"

TurboPhotonCamera::TurboPhotonCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace)
    : camera(cameraName), poseEstimator(layout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, cameraInBotSpace) {
  if constexpr (frc::RobotBase::IsSimulation()) {
    auto cameraProp = photon::SimCameraProperties();
    cameraProp.SetCalibration(1280, 720, 75_deg);
    cameraProp.SetCalibError(0.25, 0.08);
    cameraProp.SetFPS(units::hertz_t{30});
    cameraProp.SetAvgLatency(35_ms);
    cameraProp.SetLatencyStdDev(5_ms);

    systemSim.emplace("main");
    cameraSim.emplace(photon::PhotonCameraSim(&camera, cameraProp));
    cameraSim->EnableDrawWireframe(true);

    systemSim->AddAprilTags(GetLayout());
    systemSim->AddCamera(&cameraSim.value(), cameraInBotSpace);

    frc::SmartDashboard::PutData("Sim Field", &systemSim->GetDebugField());
  }

  visionTargetPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::Pose2d>(cameraName + "/targets").Publish();
}

void TurboPhotonCamera::UpdateSim(const frc::Pose2d& robotPose) {
  if constexpr (frc::RobotBase::IsSimulation()) {
    systemSim->Update(robotPose);
  }
}

const frc::AprilTagFieldLayout& TurboPhotonCamera::GetLayout() {
  try {
    layout = frc::AprilTagFieldLayout{CameraConstants::kPathToAprilTagLayout};
    frc::DataLogManager::Log("Successfully loaded edited json (April tag field layout)");
  } catch (std::exception& e) {
    frc::DataLogManager::Log("Failed to load edited json (April tag field layout): " + std::string(e.what()));
    layout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);
  }

  return layout;
}

photon::PhotonPipelineResult TurboPhotonCamera::GetLatestResult() {
  const auto results = camera.GetAllUnreadResults();

  if (results.empty()) {
    return {};
  }

  frc::DataLogManager::Log(std::to_string(results.at(0).HasTargets()));

  auto result = results.back();

  if (!result.HasTargets()) {
    return {};
  }

  std::vector<frc::Pose2d> targetPoses;

  for (const auto& target : result.GetTargets()) {
    if (target.GetFiducialId() < 0) {
      continue;
    }

    const auto tagPose = layout.GetTagPose(target.GetFiducialId());

    if (!tagPose.has_value()) {
      continue;
    }

    targetPoses.push_back(tagPose->ToPose2d());
  }

  visionTargetPublisher.Set(targetPoses);
  return result;
}

std::optional<photon::EstimatedRobotPose> TurboPhotonCamera::GetCameraEstimatedPose3D() {
  const auto result = GetLatestResult();
  return poseEstimator.Update(result);
}

std::optional<PoseTimestampPair> TurboPhotonCamera::FetchPose() {
  const auto result = GetLatestResult();

  if (const auto poseEstimate = poseEstimator.Update(result); poseEstimate.has_value() && GetNumTargets(result) >= 1) {
    return PoseTimestampPair{poseEstimate->estimatedPose.ToPose2d(), poseEstimate->timestamp};
  }

  return std::nullopt;
}

int TurboPhotonCamera::GetNumTargets() {
  const auto result = GetLatestResult();
  if (!result.HasTargets()) {
    return 0;
  }

  return static_cast<int>(result.GetTargets().size());
}
