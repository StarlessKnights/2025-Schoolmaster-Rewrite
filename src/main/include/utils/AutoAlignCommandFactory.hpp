#pragma once

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

  static std::vector<frc::Pose2d> leftBlueL4Positions;
  static std::vector<frc::Pose2d> rightBlueL4Positions;

  static std::vector<frc::Pose2d> leftRedL4Positions;
  static std::vector<frc::Pose2d> rightRedL4Positions;

 public:
  static void Initialize();
  static std::vector<frc::Pose2d> MirrorBlueSidedPoseList(const std::vector<frc::Pose2d>& inputList);
  static std::vector<frc::Pose2d> ApplyXYOffsetsToPoseList(double x, double y,
                                                           const std::vector<frc::Pose2d>& originalPoseList);

  static frc::Pose2d GetClosestScoringPose(const frc::Pose2d& currentPose, bool isRedAlliance, bool isLeftSide);
  static frc::Pose2d GetClosestL4Pose(const frc::Pose2d& currentPose, bool isRedAlliance, bool isLeftSide);
  static bool IsPoseSafeToDriveTo(const frc::Pose2d& currentPose, const frc::Pose2d& goalPose);
  static frc2::CommandPtr MakeAutoAlignDriveCommand(DriveSubsystem& drive, const frc::Pose2d& currentPosition,
                                                    bool isRedAlliance, bool isLeftSide);
  static frc2::CommandPtr MakeAutoAlignDriveCommandL4(DriveSubsystem& drive, const frc::Pose2d& currentPosition,
                                                      bool isRedAlliance, bool isLeftSide);
  static frc2::CommandPtr MakeAutoAlignAndScoreCommand(const frc::Pose2d& currentPosition, ElevatorSubsystem& elevator,
                                                       DriveSubsystem& drive, double elevatorEncoderPosition,
                                                       bool onRedAlliance, bool isLeftSide);
  static frc2::CommandPtr GetAutoAlignAndScoreCommandParallel(const frc::Pose2d& currentPosition,
                                                              ElevatorSubsystem& elevatorSubsystem,
                                                              DriveSubsystem& driveSubsystem,
                                                              double elevatorEncoderPosition, bool onRedAlliance,
                                                              bool onLeftSide);
  static frc2::CommandPtr GetL4AutoAlignCommand(const frc::Pose2d& currentPosition,
                                                ElevatorSubsystem& elevatorSubsystem, DriveSubsystem& driveSubsystem,
                                                double elevatorEncoderPosition, bool onRedAlliance, bool onLeftSide,
                                                double grabberSpeed);
  static frc2::CommandPtr GetL4AutoAlignCommandParallel(const frc::Pose2d& currentPosition,
                                                        ElevatorSubsystem& elevatorSubsystem,
                                                        DriveSubsystem& driveSubsystem, double elevatorEncoderPosition,
                                                        bool onRedAlliance, bool onLeftSide, double grabberSpeed);
};
