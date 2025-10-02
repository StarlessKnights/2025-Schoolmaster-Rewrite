// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

RobotContainer::RobotContainer() : m_driveSubsystem(), m_elevatorSubsystem() {
  ConfigureBindings();
  ConfigureElevatorBindings();
  ConfigureAlgaeGrabberBindings();
  ConfigureDefaultCommands();
}

void RobotContainer::ConfigureBindings() {
  m_driverController.Y().OnTrue(
      frc2::cmd::RunOnce([this] { m_driveSubsystem.DriverGryoZero(); }));
}

void RobotContainer::ConfigureElevatorBindings() {
  std::function<bool()> runElevatorExtruder = [this]() {
    return m_driverController.GetRightTriggerAxis() > 0.25;
  };

  // L2 Score
  m_driverController.POVLeft().OnTrue(
      ElevatorGoToPositionCommand(
          &m_elevatorSubsystem, runElevatorExtruder,
          ElevatorSubsystemConstants::kL2EncoderPosition)
          .AlongWith(SlowFieldDriveCommand(
                         &m_driveSubsystem,
                         [this] { return m_driverController.GetLeftY(); },
                         [this] { return m_driverController.GetLeftX(); },
                         [this] { return m_driverController.GetRightX(); })
                         .ToPtr()));

  // L3 Score
  m_driverController.POVUp().OnTrue(
      ElevatorGoToPositionCommand(
          &m_elevatorSubsystem, runElevatorExtruder,
          ElevatorSubsystemConstants::kL3EncoderPosition)
          .AlongWith(SlowFieldDriveCommand(
                         &m_driveSubsystem,
                         [this] { return m_driverController.GetLeftY(); },
                         [this] { return m_driverController.GetLeftX(); },
                         [this] { return m_driverController.GetRightX(); })
                         .ToPtr()));

  // L4 Score
  m_driverController.POVRight().OnTrue(
      ElevatorGoToPositionCommand(
          &m_elevatorSubsystem, runElevatorExtruder,
          ElevatorSubsystemConstants::kL4EncoderPosition)
          .AlongWith(SlowFieldDriveCommand(
                         &m_driveSubsystem,
                         [this] { return m_driverController.GetLeftY(); },
                         [this] { return m_driverController.GetLeftX(); },
                         [this] { return m_driverController.GetRightX(); })
                         .ToPtr()));

  // Cancel Command
  m_driverController.POVDown().OnTrue(frc2::cmd::Sequence(
      ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(
          &m_algaeGrabberSubsystem, &m_elevatorSubsystem,
          AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr(),
      ElevatorRetractCommand(&m_elevatorSubsystem)
          .AlongWith(
              AlgaeGrabberGoToPositionCommand(
                  &m_algaeGrabberSubsystem,
                  AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
                  .ToPtr())));

  // HP Intake
  m_driverController.RightBumper().ToggleOnTrue(
      ElevatorHPIntakeCommand(&m_elevatorSubsystem).ToPtr());
}

void RobotContainer::ConfigureAlgaeGrabberBindings() {
  std::function<bool()> runOuttake = [this]() {
    return m_driverController.GetLeftTriggerAxis() > 0.25;
  };

  m_driverController.X().OnTrue(frc2::cmd::Parallel(
      frc2::cmd::Sequence(
          AlgaeGrabberAndElevatorPositionAndIntakeCommand(
              &m_elevatorSubsystem, &m_algaeGrabberSubsystem,
              ElevatorSubsystemConstants::kHighAlgaePosition,
              AlgaeGrabberSubsystemsConstants::kAlgaeRemovalEncoderPosition)
              .ToPtr(),
          PositionHoldAndEjectCommand(&m_algaeGrabberSubsystem,
                                      &m_elevatorSubsystem, runOuttake)
              .ToPtr()),
      SlowFieldDriveCommand(
          &m_driveSubsystem, [this] { return m_driverController.GetLeftY(); },
          [this] { return m_driverController.GetLeftX(); },
          [this] { return m_driverController.GetRightX(); })
          .ToPtr()));

  m_driverController.A().OnTrue(frc2::cmd::Parallel(
      frc2::cmd::Sequence(
          AlgaeGrabberAndElevatorPositionAndIntakeCommand(
              &m_elevatorSubsystem, &m_algaeGrabberSubsystem,
              ElevatorSubsystemConstants::kLowAlgaePosition,
              AlgaeGrabberSubsystemsConstants::kAlgaeRemovalEncoderPosition)
              .ToPtr(),
          PositionHoldAndEjectCommand(&m_algaeGrabberSubsystem,
                                      &m_elevatorSubsystem, runOuttake)
              .ToPtr()),
      SlowFieldDriveCommand(
          &m_driveSubsystem, [this] { return m_driverController.GetLeftY(); },
          [this] { return m_driverController.GetLeftX(); },
          [this] { return m_driverController.GetRightX(); })
          .ToPtr()));

  m_driverController.B().OnTrue(frc2::cmd::Sequence(
      ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(
          &m_algaeGrabberSubsystem, &m_elevatorSubsystem,
          AlgaeGrabberSubsystemsConstants::kProcessorScoringEncoderPosition)
          .ToPtr(),
      UnsafeProcessorScoreCommand(&m_algaeGrabberSubsystem,
                                  &m_elevatorSubsystem, runOuttake)
          .ToPtr(),
      ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(
          &m_algaeGrabberSubsystem, &m_elevatorSubsystem,
          AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr()));
}

void RobotContainer::ConfigureDefaultCommands() {
  m_driveSubsystem.SetDefaultCommand(GetDefaultDriveCommand());
  m_elevatorSubsystem.SetDefaultCommand(
      ElevatorRetractCommand(&m_elevatorSubsystem).ToPtr());
  m_algaeGrabberSubsystem.SetDefaultCommand(
      AlgaeGrabberGoToPositionCommand(
          &m_algaeGrabberSubsystem,
          AlgaeGrabberSubsystemsConstants::kRetractedEncoderPosition)
          .ToPtr());
}

frc2::CommandPtr RobotContainer::GetDefaultDriveCommand() {
  return FieldDriveCommand(
             &m_driveSubsystem,
             [this] { return m_driverController.GetLeftY(); },
             [this] { return m_driverController.GetLeftX(); },
             [this] { return m_driverController.GetRightX(); })
      .ToPtr();
}
