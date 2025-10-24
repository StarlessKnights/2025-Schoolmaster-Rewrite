// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "utils/AutoAlignCommandFactory.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "commands/autoalign/FollowPrecisePathCommand.hpp"
#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/InstantCommand.h"
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

std::vector<frc::Pose2d> AutoAlignCommandFactory::ApplyXYOffset(const std::vector<frc::Pose2d>& originalPoses,
                                                                const double x, const double y) {
  std::vector<frc::Pose2d> adjustedPoses;

  for (const auto& pose : originalPoses) {
    const double offset = std::atan2(x, y);
    const double magOffset = std::sqrt(x * x + y * y);
    const auto poseOrientation = pose.Rotation().Radians().to<double>();

    const double newX = pose.X().to<double>() + (std::cos(poseOrientation + offset) * magOffset);
    const double newY = pose.Y().to<double>() + (std::sin(poseOrientation + offset) * magOffset);

    adjustedPoses.emplace_back(units::meter_t{newX}, units::meter_t{newY}, pose.Rotation());
  }

  return adjustedPoses;
}

std::vector<frc::Pose2d> AutoAlignCommandFactory::MirrorBlueSidedPoseList(const std::vector<frc::Pose2d>& poseList) {
  std::vector<frc::Pose2d> redPoses;

  for (const auto& pose : poseList) {
    const double mirroredX = PathingConstants::kFieldWidthMeters - pose.X().to<double>();
    const double mirroredY = PathingConstants::kFieldLengthMeters - pose.Y().to<double>();
    const auto mirroredRotation = pose.Rotation().RotateBy(frc::Rotation2d{180_deg}).Radians().to<double>();

    redPoses.emplace_back(units::meter_t{mirroredX}, units::meter_t{mirroredY}, units::radian_t{mirroredRotation});
  }

  return redPoses;
}

frc::Pose2d AutoAlignCommandFactory::GetClosestScoringPose(const frc::Pose2d& currentPose, const bool isRedAlliance,
                                                           const bool isLeftSide) {
  Initialize();

  std::vector<frc::Pose2d> poseList =
      isRedAlliance ? (isLeftSide ? leftRedAllianceScoringPositions : rightRedAllianceScoringPositions)
                    : (isLeftSide ? leftBlueAllianceScoringPositions : rightBlueAllianceScoringPositions);

  return currentPose.Nearest(poseList);
}

bool AutoAlignCommandFactory::IsPoseSafeToDriveTo(const frc::Pose2d& currentPose, const frc::Pose2d& goalPose) {
  const double distSquared = std::pow(currentPose.X().value() - goalPose.X().value(), 2) +
                             std::pow(currentPose.Y().value() - goalPose.Y().value(), 2);

  frc::DataLogManager::Log("Distance to target: " + std::to_string(std::sqrt(distSquared)));

  return std::sqrt(distSquared) < PathingConstants::kMaxPathingDistance;
}
bool AutoAlignCommandFactory::IsPoseSafeToDriveTo(const frc::Pose2d& currentPose, const frc::Pose2d& goalPose,
                                                  const double maxSafeDistance) {
  const double distSquared = std::pow(currentPose.X().value() - goalPose.X().value(), 2) +
                             std::pow(currentPose.Y().value() - goalPose.Y().value(), 2);

  frc::DataLogManager::Log("Distance to target: " + std::to_string(distSquared));

  return std::sqrt(distSquared) < maxSafeDistance;
}

frc2::CommandPtr AutoAlignCommandFactory::MakeAutoAlignAndScoreCommand(const std::function<frc::Pose2d()>& poseSupplier,
                                                                       ElevatorSubsystem* elevator,
                                                                       DriveSubsystem* drive,
                                                                       const double elevatorEncoderPosition,
                                                                       const std::function<bool()>& isRedAlliance,
                                                                       const std::function<bool()>& isLeftSide) {
  Initialize();

  auto goalSupplier = [poseSupplier, isRedAlliance, isLeftSide]() {
    return GetClosestScoringPose(poseSupplier(), isRedAlliance(), isLeftSide());
  };

  return FollowPrecisePathCommand(drive, goalSupplier)
      .AlongWith(elevator->MoveElevatorToPositionCommand(elevatorEncoderPosition))
      .AndThen(frc2::InstantCommand([elevator] {
                 elevator->SetCoralGrabber(ElevatorSubsystemConstants::kGrabberSpeed);
               }).ToPtr())
      .OnlyIf([poseSupplier, goalSupplier] { return IsPoseSafeToDriveTo(poseSupplier(), goalSupplier()); });
}

frc2::CommandPtr AutoAlignCommandFactory::MakeAutoProcessorScoreCommand(
    DriveSubsystem* drive, const std::function<frc::Pose2d()>& poseSupplier) {
  Initialize();

  auto goalSupplier = []() { return PathingConstants::kProcessorBlueScoringPosition; };

  return FollowPrecisePathCommand(drive, goalSupplier).OnlyIf([poseSupplier, goalSupplier] {
    return IsPoseSafeToDriveTo(poseSupplier(), goalSupplier(), PathingConstants::kMaxProcessorScoringDistance);
      })
      .WithName("Processor Auto Align");
}
