#include "utils/AutoAlignCommandFactory.hpp"
#include <cmath>
#include <vector>
#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "units/length.h"

bool AutoAlignCommandFactory::initialized = false;
std::vector<frc::Pose2d> AutoAlignCommandFactory::leftBlueAllianceScoringPositions;
std::vector<frc::Pose2d> AutoAlignCommandFactory::rightBlueAllianceScoringPositions;
std::vector<frc::Pose2d> AutoAlignCommandFactory::leftRedAllianceScoringPositions;
std::vector<frc::Pose2d> AutoAlignCommandFactory::rightRedAllianceScoringPositions;

void AutoAlignCommandFactory::Initialize() {
  if (!initialized) {
    initialized = true;

    leftBlueAllianceScoringPositions = ApplyXYOffset(PathingConstants::kLeftBlueSidedScoringPositions,
                                                     PathingConstants::kXOffset, PathingConstants::kYOffset);
    rightBlueAllianceScoringPositions = ApplyXYOffset(PathingConstants::kRightBlueSidedScoringPositions,
                                                      PathingConstants::kXOffset, PathingConstants::kYOffset);

    leftRedAllianceScoringPositions = MirrorBlueSidedPoseList(leftBlueAllianceScoringPositions);
    rightRedAllianceScoringPositions = MirrorBlueSidedPoseList(rightBlueAllianceScoringPositions);
  }
}

std::vector<frc::Pose2d> AutoAlignCommandFactory::ApplyXYOffset(const std::vector<frc::Pose2d>& originalPoses, double x,
                                                                double y) {
  std::vector<frc::Pose2d> adjustedPoses;

  for (const auto& pose : originalPoses) {
    double offset = std::atan2(x, y);
    double magOffset = std::sqrt(x * x + y * y);
    double poseOrientation = pose.Rotation().Radians().to<double>();

    double newX = pose.X().to<double>() + (std::cos(poseOrientation + offset) * magOffset);
    double newY = pose.Y().to<double>() + (std::sin(poseOrientation + offset) * magOffset);

    adjustedPoses.emplace_back(units::meter_t{newX}, units::meter_t{newY}, pose.Rotation());
  }

  return adjustedPoses;
}

std::vector<frc::Pose2d> AutoAlignCommandFactory::MirrorBlueSidedPoseList(const std::vector<frc::Pose2d>& bluePoses) {
  std::vector<frc::Pose2d> redPoses;

  for (const auto& pose : bluePoses) {
    double mirroredX = PathingConstants::kFieldWidthMeters - pose.X().to<double>();
    double mirroredY = PathingConstants::kFieldLengthMeters - pose.Y().to<double>();
    double mirroredRotation = pose.Rotation().RotateBy(frc::Rotation2d{180_deg}).Radians().to<double>();

    redPoses.emplace_back(units::meter_t{mirroredX}, units::meter_t{mirroredY}, units::radian_t{mirroredRotation});
  }

  return redPoses;
}

frc::Pose2d AutoAlignCommandFactory::GetClosestScoringPose(const frc::Pose2d& currentPose, bool isRedAlliance,
                                                           bool isLeftSide) {
  Initialize();

  frc::DataLogManager::Log("Getting closest scoring pose for " + std::string(isRedAlliance ? "Red" : "Blue") +
                           " alliance, " + std::string(isLeftSide ? "Left" : "Right") + " side");
  frc::DataLogManager::Log("Current pose: X=" + std::to_string(currentPose.X().to<double>()) +
                           ", Y=" + std::to_string(currentPose.Y().to<double>()) +
                           ", Rotation=" + std::to_string(currentPose.Rotation().Degrees().to<double>()));

  std::vector<frc::Pose2d> poseList =
      isRedAlliance ? (isLeftSide ? leftRedAllianceScoringPositions : rightRedAllianceScoringPositions)
                    : (isLeftSide ? leftBlueAllianceScoringPositions : rightBlueAllianceScoringPositions);

  auto nearest = currentPose.Nearest(poseList);

  frc::DataLogManager::Log("Nearest pose: X=" + std::to_string(nearest.X().to<double>()) +
                           ", Y=" + std::to_string(nearest.Y().to<double>()) +
                           ", Rotation=" + std::to_string(nearest.Rotation().Degrees().to<double>()));

  return nearest;
}
