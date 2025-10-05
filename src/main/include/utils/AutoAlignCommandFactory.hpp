#pragma once

#include <functional>
#include <vector>
#include "frc/geometry/Pose2d.h"
#include "frc2/command/CommandPtr.h"
#include "subsystems/DriveSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"

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
  static bool IsPoseSafeToDriveTo(const frc::Pose2d& currentPose, const frc::Pose2d& goalPose);

  static frc2::CommandPtr MakeAutoAlignAndScoreCommand(std::function<frc::Pose2d()> poseSupplier,
                                                       ElevatorSubsystem* elevator, DriveSubsystem* drive,
                                                       double elevatorEncoderPosition, bool isRedAlliance,
                                                       bool isLeftSide);
};
