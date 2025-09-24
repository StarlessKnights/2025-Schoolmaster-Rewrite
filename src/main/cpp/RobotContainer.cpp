// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/FieldDriveCommand.hpp"
#include "commands/elevator/ElevatorGoToPositionCommand.hpp"
#include "commands/elevator/ElevatorRetractCommand.hpp"
#include "constants/Constants.h"

RobotContainer::RobotContainer() : m_driveSubsystem(), m_elevatorSubsystem() {
  ConfigureBindings();
  ConfigureElevatorBindings();
  ConfigureDefaultCommands();
}

void RobotContainer::ConfigureBindings() {}

void RobotContainer::ConfigureElevatorBindings() {
  m_driverController.Button(1).OnTrue(
      ElevatorGoToPositionCommand(&m_elevatorSubsystem, ElevatorSubsystemConstants::kL4EncoderPosition).ToPtr());
  m_driverController.Button(2).OnTrue(
      ElevatorGoToPositionCommand(&m_elevatorSubsystem, ElevatorSubsystemConstants::kL3EncoderPosition).ToPtr());
  m_driverController.Button(3).OnTrue(ElevatorRetractCommand(&m_elevatorSubsystem).ToPtr());
}

void RobotContainer::ConfigureDefaultCommands() { m_driveSubsystem.SetDefaultCommand(GetDefaultDriveCommand()); }

frc2::CommandPtr RobotContainer::GetDefaultDriveCommand() {
  return FieldDriveCommand(
             &m_driveSubsystem, [this] { return m_driverController.GetLeftY(); },
             [this] { return m_driverController.GetLeftX(); }, [this] { return m_driverController.GetRightX(); })
      .ToPtr();
}