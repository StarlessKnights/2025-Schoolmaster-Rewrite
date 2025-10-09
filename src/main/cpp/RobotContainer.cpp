// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <functional>
#include <string>
#include <vector>

#include "commands/FieldDriveCommand.hpp"
#include "commands/SlowFieldDriveCommand.hpp"
#include "commands/algaegrabber/AlgaeGrabberAndElevatorPositionAndIntakeCommand.hpp"
#include "commands/algaegrabber/AlgaeGrabberGoToPositionCommand.hpp"
#include "commands/algaegrabber/ElevatorPopUpAndAlgaeGrabberGoToPositionCommand.hpp"
#include "commands/algaegrabber/PositionHoldAndEjectCommand.hpp"
#include "commands/algaegrabber/UnsafeProcessorScoreCommand.hpp"
#include "commands/elevator/ElevatorGoToPositionCommand.hpp"
#include "commands/elevator/ElevatorHPIntakeCommand.hpp"
#include "commands/elevator/ElevatorRetractCommand.hpp"
#include "commands/elevator/autonomous/ExtendToHeightThenScoreCommand.hpp"
#include "commands/led/IndicateSideCommand.hpp"
#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "utils/AutoAlignCommandFactory.hpp"
#include "utils/PathLoader.hpp"

RobotContainer::RobotContainer() : m_driveSubsystem(), m_elevatorSubsystem(), m_ledSubsystem() {
  ConfigureNamedCommands();
  ConfigureBindings();
  ConfigureElevatorBindings();
  ConfigureAlgaeGrabberBindings();
  ConfigureManualOverrideBindings();
  ConfigureDefaultCommands();

  PathLoader::ConfigurePathPlanner(m_driveSubsystem, m_driveSubsystem.GetPoseEstimator());
  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](std::vector<frc::Pose2d> poses) { m_pathPosesPublisher.Set(poses); });

  frc::SmartDashboard::PutData(&m_driveSubsystem);
  frc::SmartDashboard::PutData(&m_elevatorSubsystem);
  frc::SmartDashboard::PutData(&m_algaeGrabberSubsystem);
  frc::SmartDashboard::PutData(&m_ledSubsystem);
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}

void RobotContainer::ConfigureBindings() {
  m_driverController.Y().OnTrue(
      frc2::cmd::RunOnce([this] { m_driveSubsystem.DriverGryoZero(); }).WithName("Reset Gyro"));
}

void RobotContainer::ConfigureElevatorBindings() {
  // Outtake
  std::function<bool()> runElevatorExtruder = [this]() { return m_driverController.GetRightTriggerAxis() > 0.25; };
  std::function<bool()> isManuallyOverridden = [this]() { return this->isManuallyOverridden; };
  std::function<bool()> scoringOnLeftProvider = [this]() { return this->scoringOnLeft; };
  std::function<bool()> isRedAlliance = []() { return frc::DriverStation::GetAlliance() == frc::DriverStation::kRed; };

  // L2 Score
  m_driverController.POVRight().OnTrue(
      frc2::cmd::Either(MakeElevatorScoreSequence(ElevatorSubsystemConstants::kL2EncoderPosition, runElevatorExtruder)
                            .WithName("L2 Manual Score"),
                        AutoAlignCommandFactory::MakeAutoAlignAndScoreCommand(
                            [&] { return m_driveSubsystem.GetPose(); }, &m_elevatorSubsystem, &m_driveSubsystem,
                            ElevatorSubsystemConstants::kL2EncoderPosition, isRedAlliance, scoringOnLeftProvider),
                        isManuallyOverridden)
          .AndThen(frc2::cmd::WaitUntil([this]() { return m_driverController.POVDown().Get(); }))
          .WithName("L2 Score"));

  // L3 Score
  m_driverController.POVUp().OnTrue(
      frc2::cmd::Either(MakeElevatorScoreSequence(ElevatorSubsystemConstants::kL3EncoderPosition, runElevatorExtruder)
                            .WithName("L3 Manual Score"),
                        AutoAlignCommandFactory::MakeAutoAlignAndScoreCommand(
                            [&] { return m_driveSubsystem.GetPose(); }, &m_elevatorSubsystem, &m_driveSubsystem,
                            ElevatorSubsystemConstants::kL3EncoderPosition, isRedAlliance, scoringOnLeftProvider),
                        isManuallyOverridden)
          .AndThen(frc2::cmd::WaitUntil([this]() { return m_driverController.POVDown().Get(); }))
          .WithName("L3 Score"));

  // L4 Score
  m_driverController.POVLeft().OnTrue(
      frc2::cmd::Either(MakeElevatorScoreSequence(ElevatorSubsystemConstants::kL4EncoderPosition, runElevatorExtruder)
                            .WithName("L4 Manual Score"),
                        AutoAlignCommandFactory::MakeAutoAlignAndScoreCommand(
                            [&] { return m_driveSubsystem.GetPose(); }, &m_elevatorSubsystem, &m_driveSubsystem,
                            ElevatorSubsystemConstants::kL4EncoderPosition, isRedAlliance, scoringOnLeftProvider),
                        isManuallyOverridden)
          .AndThen(frc2::cmd::WaitUntil([this]() { return m_driverController.POVDown().Get(); }))
          .WithName("L4 Score"));

  // Cancel Command
  m_driverController.POVDown().OnTrue(MakeCancelCommand());

  // HP Intake
  m_driverController.RightBumper().ToggleOnTrue(
      ElevatorHPIntakeCommand(&m_elevatorSubsystem).WithName("ElevatorHPIntakeCommand"));
}

void RobotContainer::ConfigureAlgaeGrabberBindings() {
  // Outtake
  std::function<bool()> runOuttake = [this]() { return m_driverController.GetLeftTriggerAxis() > 0.25; };

  // High Algae
  m_driverController.X().OnTrue(
      frc2::cmd::Parallel(MakeAlgaeGrabberSequence(ElevatorSubsystemConstants::kHighAlgaePosition, runOuttake),
                          MakeSlowFieldDriveCommand())
          .WithName("HighAlgaeCommand"));

  // Low Algae
  m_driverController.A().OnTrue(
      frc2::cmd::Parallel(MakeAlgaeGrabberSequence(ElevatorSubsystemConstants::kLowAlgaePosition, runOuttake),
                          MakeSlowFieldDriveCommand())
          .WithName("LowAlgaeCommand"));

  // Processor Score
  m_driverController.B().OnTrue(MakeProcessorScoreSequence(runOuttake).WithName("ProcessorScoreCommand"));
}

void RobotContainer::ConfigureManualOverrideBindings() {
  m_driverController.Back().OnTrue(frc2::cmd::RunOnce([this] {
                                     isManuallyOverridden = !isManuallyOverridden;
                                     frc::DataLogManager::Log("Manual Override changed to " +
                                                              std::string(isManuallyOverridden ? "yes" : "no"));
                                   }).WithName("ToggleManualOverride"));
  m_driverController.Start().OnTrue(frc2::cmd::RunOnce([this] {
                                      scoringOnLeft = !scoringOnLeft;
                                      frc::DataLogManager::Log("Scoring side changed to " +
                                                               std::string(scoringOnLeft ? "left" : "right"));
                                    }).WithName("ChangeScoringSide"));
}

void RobotContainer::ConfigureDefaultCommands() {
  std::function<bool()> scoring = [this]() { return this->scoringOnLeft; };
  std::function<bool()> isManuallyOverridden = [this]() { return this->isManuallyOverridden; };

  m_driveSubsystem.SetDefaultCommand(MakeFieldDriveCommand());
  m_elevatorSubsystem.SetDefaultCommand(ElevatorRetractCommand(&m_elevatorSubsystem).ToPtr());
  m_algaeGrabberSubsystem.SetDefaultCommand(
      AlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem,
                                      AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr());
  m_ledSubsystem.SetDefaultCommand(IndicateSideCommand(&m_ledSubsystem, scoring, isManuallyOverridden));
}

void RobotContainer::ConfigureNamedCommands() {
  pathplanner::NamedCommands::registerCommand(
      "ScoreL4", ExtendToHeightThenScoreCommand(&m_elevatorSubsystem, ElevatorSubsystemConstants::kL4EncoderPosition)
                     .WithTimeout(2.5_s));
  pathplanner::NamedCommands::registerCommand("HPIntake", ElevatorHPIntakeCommand(&m_elevatorSubsystem).ToPtr());
}

frc2::CommandPtr RobotContainer::MakeFieldDriveCommand() {
  return FieldDriveCommand(
             &m_driveSubsystem, [this] { return m_driverController.GetLeftY(); },
             [this] { return m_driverController.GetLeftX(); }, [this] { return m_driverController.GetRightX(); })
      .ToPtr();
}

frc2::CommandPtr RobotContainer::MakeAlgaeGrabberSequence(double elevatorPosition, std::function<bool()> runExtruder) {
  return frc2::cmd::Sequence(
      AlgaeGrabberAndElevatorPositionAndIntakeCommand(&m_elevatorSubsystem, &m_algaeGrabberSubsystem, elevatorPosition,
                                                      AlgaeGrabberSubsystemsConstants::kAlgaeRemovalEncoderPosition)
          .ToPtr(),
      PositionHoldAndEjectCommand(&m_algaeGrabberSubsystem, &m_elevatorSubsystem, runExtruder).ToPtr());
}

frc2::CommandPtr RobotContainer::MakeSlowFieldDriveCommand() {
  return SlowFieldDriveCommand(
             &m_driveSubsystem, [this] { return m_driverController.GetLeftY(); },
             [this] { return m_driverController.GetLeftX(); }, [this] { return m_driverController.GetRightX(); })
      .ToPtr();
}

frc2::CommandPtr RobotContainer::MakeProcessorScoreSequence(std::function<bool()> runOuttake) {
  return frc2::cmd::Sequence(
      ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem, &m_elevatorSubsystem,
                                                      AlgaeGrabberSubsystemsConstants::kProcessorScoringEncoderPosition)
          .ToPtr(),
      UnsafeProcessorScoreCommand(&m_algaeGrabberSubsystem, &m_elevatorSubsystem, runOuttake).ToPtr(),
      ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem, &m_elevatorSubsystem,
                                                      AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr());
}

frc2::CommandPtr RobotContainer::MakeElevatorScoreSequence(double elevatorPosition, std::function<bool()> runExtruder) {
  return frc2::cmd::Parallel(ElevatorGoToPositionCommand(&m_elevatorSubsystem, runExtruder, elevatorPosition).ToPtr(),
                             MakeSlowFieldDriveCommand());
}

frc2::CommandPtr RobotContainer::MakeCancelCommand() {
  return frc2::cmd::Sequence(
             ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem, &m_elevatorSubsystem,
                                                             AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
                 .ToPtr(),
             ElevatorRetractCommand(&m_elevatorSubsystem)
                 .AlongWith(AlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem,
                                                            AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
                                .ToPtr()))
      .WithName("ResetAfterMovementCommand");
}
