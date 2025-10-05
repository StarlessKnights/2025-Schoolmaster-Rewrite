// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <functional>

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
#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "utils/AutoAlignCommandFactory.hpp"

RobotContainer::RobotContainer() : m_driveSubsystem(), m_elevatorSubsystem() {
  ConfigureBindings();
  ConfigureElevatorBindings();
  ConfigureAlgaeGrabberBindings();
  ConfigureManualOverrideBindings();
  ConfigureDefaultCommands();

  frc::SmartDashboard::PutBoolean("Manual Override", isManuallyOverridden);

  frc::SmartDashboard::PutData(&m_driveSubsystem);
  frc::SmartDashboard::PutData(&m_elevatorSubsystem);
  frc::SmartDashboard::PutData(&m_algaeGrabberSubsystem);
}

void RobotContainer::ConfigureBindings() {
  m_driverController.Y().OnTrue(
      frc2::cmd::RunOnce([this] { m_driveSubsystem.DriverGryoZero(); }).WithName("Reset Gyro"));
}

void RobotContainer::ConfigureElevatorBindings() {
  // Outtake
  std::function<bool()> runElevatorExtruder = [this]() { return m_driverController.GetRightTriggerAxis() > 0.25; };
  std::function<bool()> isManuallyOverridden = [this]() { return this->isManuallyOverridden; };

  // L2 Score
  m_driverController.Button(1).OnTrue(
      frc2::cmd::Either(MakeElevatorScoreSequence(ElevatorSubsystemConstants::kL2EncoderPosition, runElevatorExtruder)
                            .AlongWith(MakeSlowFieldDriveCommand())
                            .WithName("L2 Manual Score"),
                        AutoAlignCommandFactory::MakeAutoAlignAndScoreCommand(
                            [&] { return m_driveSubsystem.GetSimPose(); }, &m_elevatorSubsystem, &m_driveSubsystem,
                            ElevatorSubsystemConstants::kL2EncoderPosition, true, true),
                        isManuallyOverridden)
          .WithName("L2 Score"));

  // L3 Score
  m_driverController.Button(2).OnTrue(
      frc2::cmd::Either(MakeElevatorScoreSequence(ElevatorSubsystemConstants::kL3EncoderPosition, runElevatorExtruder)
                            .AlongWith(MakeSlowFieldDriveCommand())
                            .WithName("L3 Manual Score"),
                        AutoAlignCommandFactory::MakeAutoAlignAndScoreCommand(
                            [&] { return m_driveSubsystem.GetSimPose(); }, &m_elevatorSubsystem, &m_driveSubsystem,
                            ElevatorSubsystemConstants::kL3EncoderPosition, true, true),
                        isManuallyOverridden)
          .WithName("L3 Score"));

  // L4 Score
  m_driverController.POVRight().OnTrue(
      frc2::cmd::Either(MakeElevatorScoreSequence(ElevatorSubsystemConstants::kL4EncoderPosition, runElevatorExtruder)
                            .AlongWith(MakeSlowFieldDriveCommand())
                            .WithName("L4 Manual Score"),
                        AutoAlignCommandFactory::MakeAutoAlignAndScoreCommand(
                            [&] { return m_driveSubsystem.GetSimPose(); }, &m_elevatorSubsystem, &m_driveSubsystem,
                            ElevatorSubsystemConstants::kL4EncoderPosition, true, true),
                        isManuallyOverridden)
          .WithName("L4 Score"));

  // Cancel Command
  m_driverController.POVDown().OnTrue(MakeCancelCommand());

  // HP Intake
  m_driverController.RightBumper().ToggleOnTrue(ElevatorHPIntakeCommand(&m_elevatorSubsystem).ToPtr());
}

void RobotContainer::ConfigureAlgaeGrabberBindings() {
  // Outtake
  std::function<bool()> runOuttake = [this]() { return m_driverController.GetLeftTriggerAxis() > 0.25; };

  // High Algae
  m_driverController.X().OnTrue(
      frc2::cmd::Parallel(MakeAlgaeGrabberSequence(ElevatorSubsystemConstants::kHighAlgaePosition, runOuttake),
                          MakeSlowFieldDriveCommand()));

  // Low Algae
  // m_driverController.A().OnTrue(
  //     frc2::cmd::Parallel(MakeAlgaeGrabberSequence(ElevatorSubsystemConstants::kLowAlgaePosition, runOuttake),
  //                         MakeSlowFieldDriveCommand()));

  // Processor Score
  m_driverController.B().OnTrue(MakeProcessorScoreSequence(runOuttake));
}

void RobotContainer::ConfigureManualOverrideBindings() {
  m_driverController.Back().OnTrue(frc2::cmd::RunOnce([this] {
                                     isManuallyOverridden = !isManuallyOverridden;
                                     frc::DataLogManager::Log("Manual Override changed");
                                   }).WithName("Toggle Manual Override"));
}

void RobotContainer::ConfigureDefaultCommands() {
  m_driveSubsystem.SetDefaultCommand(MakeFieldDriveCommand());
  m_elevatorSubsystem.SetDefaultCommand(ElevatorRetractCommand(&m_elevatorSubsystem).ToPtr());
  m_algaeGrabberSubsystem.SetDefaultCommand(
      AlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem,
                                      AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr());
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
  return frc2::cmd::Sequence(
      ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem, &m_elevatorSubsystem,
                                                      AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr(),
      ElevatorGoToPositionCommand(&m_elevatorSubsystem, runExtruder, elevatorPosition).ToPtr());
}

frc2::CommandPtr RobotContainer::MakeCancelCommand() {
  return frc2::cmd::Sequence(
      ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem, &m_elevatorSubsystem,
                                                      AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr(),
      ElevatorRetractCommand(&m_elevatorSubsystem)
          .AlongWith(AlgaeGrabberGoToPositionCommand(&m_algaeGrabberSubsystem,
                                                     AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
                         .ToPtr()));
}
