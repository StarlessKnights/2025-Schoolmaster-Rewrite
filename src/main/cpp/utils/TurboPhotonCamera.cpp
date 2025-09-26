#include "utils/TurboPhotonCamera.hpp"
#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/Timer.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform3d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/simulation/PhotonCameraSim.h"
#include "photon/simulation/SimCameraProperties.h"
#include "photon/simulation/VisionSystemSim.h"
#include "photon/targeting/PhotonPipelineResult.h"
#include "units/frequency.h"
#include "units/time.h"
#include "utils/PoseTimestampPair.hpp"
#include <exception>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

TurboPhotonCamera::TurboPhotonCamera(const std::string &cameraName, frc::Transform3d cameraInBotSpace)
    : layout(getLayout()), camera(cameraName),
      poseEstimator(layout, photon::MULTI_TAG_PNP_ON_COPROCESSOR, cameraInBotSpace) {
  if (frc::RobotBase::IsSimulation()) {
    auto cameraProp = photon::SimCameraProperties();
    cameraProp.SetCalibration(1280, 720, 75_deg);
    cameraProp.SetCalibError(0.25, 0.08);
    cameraProp.SetFPS(units::hertz_t{30});
    cameraProp.SetAvgLatency(35_ms);
    cameraProp.SetLatencyStdDev(5_ms);

    systemSim.emplace("main");
    cameraSim.emplace(photon::PhotonCameraSim(&camera, cameraProp));
    cameraSim->EnableDrawWireframe(true);

    systemSim->AddAprilTags(getLayout());
    systemSim->AddCamera(&cameraSim.value(), cameraInBotSpace);
  }

  visionTargetPublisher =
      nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::Pose2d>(cameraName + "/targets").Publish();
}

void TurboPhotonCamera::updateSim(frc::Pose2d robotPose) {
  if (frc::RobotBase::IsSimulation()) {
    frc::SmartDashboard::PutData("Sim Field", &systemSim->GetDebugField());
    systemSim->Update(robotPose);
  }
}

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
  auto result = camera.GetLatestResult();
  std::vector<frc::Pose2d> targetPoses;

  for (const auto &target : result.GetTargets()) {
    if (target.GetFiducialId() >= 0) {
      auto tagPose = layout.GetTagPose(target.GetFiducialId());
      if (tagPose.has_value()) {
        targetPoses.push_back(tagPose->ToPose2d());
      }
    }
  }

  visionTargetPublisher.Set(targetPoses);

  return result;
}

std::optional<photon::EstimatedRobotPose> TurboPhotonCamera::getCameraEstimatedPose3d() {
  auto result = getLatestResult();

  frc::SmartDashboard::PutNumber("Number of targets", result.GetTargets().size());
  frc::SmartDashboard::PutNumber("Best target ID", result.GetBestTarget().GetFiducialId());
  frc::SmartDashboard::PutNumber("Timestamp", double(result.GetTimestamp().value()));

  if (result.GetTimestamp() == units::second_t{-1}) {
    frc::DataLogManager::Log("Photon result not valid");
    return std::nullopt;
  }

  return poseEstimator.Update(result);
}

std::optional<PoseTimestampPair> TurboPhotonCamera::fetchPose() {
  std::optional<photon::EstimatedRobotPose> ret = std::nullopt;

  try {
    ret = getCameraEstimatedPose3d();
  } catch (std::exception &e) {
    frc::DataLogManager::Log("Failed to get camera estimated pose: " + std::string(e.what()));
    return std::nullopt;
  }

  if (ret.has_value() && getLatestResult().GetTargets().size() >= 1) {
    units::second_t currentTime = frc::Timer::GetFPGATimestamp();
    units::second_t poseTime = ret->timestamp;

    frc::SmartDashboard::PutNumber("photon pose X", ret->estimatedPose.X().to<double>());
    frc::SmartDashboard::PutNumber("photon pose Y", ret->estimatedPose.Y().to<double>());
    frc::SmartDashboard::PutNumber("photon pose Rotation",
                                   ret->estimatedPose.ToPose2d().Rotation().Degrees().to<double>());
    frc::SmartDashboard::PutNumber("photon pose Timestamp", poseTime.to<double>());
    frc::SmartDashboard::PutNumber("photon current Time", currentTime.to<double>());
    frc::SmartDashboard::PutNumber("photon time Diff", (currentTime - poseTime).to<double>());

    if (currentTime.value() - poseTime.value() > 0.5) {
      frc::DataLogManager::Log("Photon pose too late, Time Diff: " +
                               std::to_string(currentTime.value() - poseTime.value()));
      return std::nullopt;
    }

    if (poseTime.value() == -1) {
      frc::DataLogManager::Log("Photon pose has invalid timestamp");
      return std::nullopt;
    }

    return PoseTimestampPair{ret->estimatedPose.ToPose2d(), poseTime};
  }

  return std::nullopt;
}
