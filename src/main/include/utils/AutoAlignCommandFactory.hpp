#pragma once

#include <vector>
#include "frc/geometry/Pose2d.h"

class AutoAlignCommandFactory {
 private:
  static std::vector<frc::Pose2d> leftBlueAllianceScoringPositions;
  static std::vector<frc::Pose2d> rightBlueAllianceScoringPositions;

  static std::vector<frc::Pose2d> leftRedAllianceScoringPositions;
  static std::vector<frc::Pose2d> rightRedAllianceScoringPositions;

  static bool initialized;

  static std::vector<frc::Pose2d> MirrorBlueSidedPoseList(const std::vector<frc::Pose2d>& list);
  static std::vector<frc::Pose2d> ApplyXYOffset(const std::vector<frc::Pose2d>& list, double x, double y);

 public:
  static void Initialize();

  static frc::Pose2d GetClosestScoringPose(const frc::Pose2d& currentPose, bool isRedAlliance, bool isLeftSide);
};
