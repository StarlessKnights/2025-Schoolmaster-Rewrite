// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/FieldDriveCommand.hpp"
#include "commands/algaegrabber/AlgaeGrabberGoToPositionCommand.hpp"
#include "commands/algaegrabber/ElevatorPopUpAndAlgaeGrabberGoToPositionCommand.hpp"
#include "commands/elevator/ElevatorGoToPositionCommand.hpp"
#include "commands/elevator/ElevatorRetractCommand.hpp"
#include "constants/Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

RobotContainer::RobotContainer() : m_driveSubsystem(), m_elevatorSubsystem() {
  ConfigureBindings();
  ConfigureElevatorBindings();
  ConfigureDefaultCommands();
}

void RobotContainer::ConfigureBindings() {
  m_driverController.Y().OnTrue(
      frc2::cmd::RunOnce([this] { m_driveSubsystem.DriverGryoZero(); }));
}

void RobotContainer::ConfigureElevatorBindings() {
  // L2 Score
  m_driverController.POVLeft().OnTrue(
      ElevatorGoToPositionCommand(
          &m_elevatorSubsystem, ElevatorSubsystemConstants::kL2EncoderPosition)
          .ToPtr());

  // L3 Score
  m_driverController.POVUp().OnTrue(
      ElevatorGoToPositionCommand(
          &m_elevatorSubsystem, ElevatorSubsystemConstants::kL3EncoderPosition)
          .ToPtr());

  // L4 Score
  m_driverController.POVRight().OnTrue(
      ElevatorGoToPositionCommand(
          &m_elevatorSubsystem, ElevatorSubsystemConstants::kL4EncoderPosition)
          .ToPtr());

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
