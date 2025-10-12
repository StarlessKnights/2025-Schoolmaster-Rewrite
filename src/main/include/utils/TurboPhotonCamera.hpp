// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <optional>
#include <string>

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

class TurboPhotonCamera {
 private:
  frc::AprilTagFieldLayout layout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2025ReefscapeAndyMark);
  photon::PhotonCamera camera;
  photon::PhotonPoseEstimator poseEstimator;

  std::optional<photon::VisionSystemSim> systemSim = std::nullopt;
  std::optional<photon::PhotonCameraSim> cameraSim = std::nullopt;

  nt::StructArrayPublisher<frc::Pose2d> visionTargetPublisher;

 public:
  TurboPhotonCamera(const std::string& cameraName, frc::Transform3d cameraInBotSpace);
  void UpdateSim(frc::Pose2d robotPose);
  const frc::AprilTagFieldLayout& GetLayout();
  photon::PhotonPipelineResult GetLatestResult();
  std::optional<photon::EstimatedRobotPose> GetCameraEstimatedPose3D();
  std::optional<PoseTimestampPair> FetchPose();
  int GetNumTargets();
  int GetNumTargets(const photon::PhotonPipelineResult& result) { return result.GetTargets().size(); }
};
