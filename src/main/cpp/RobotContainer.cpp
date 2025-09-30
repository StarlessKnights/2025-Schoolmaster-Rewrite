// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/FieldDriveCommand.hpp"
#include "commands/elevator/ElevatorRetractCommand.hpp"
#include "frc2/command/Commands.h"

RobotContainer::RobotContainer() : m_driveSubsystem(), m_elevatorSubsystem() {
  ConfigureBindings();
  ConfigureDefaultCommands();
}

void RobotContainer::ConfigureBindings() {
  m_driverController.Y().OnTrue(frc2::cmd::RunOnce([this] { m_driveSubsystem.DriverGryoZero(); }));
}

void RobotContainer::ConfigureDefaultCommands() {
  m_driveSubsystem.SetDefaultCommand(GetDefaultDriveCommand());
  m_elevatorSubsystem.SetDefaultCommand(ElevatorRetractCommand(&m_elevatorSubsystem).ToPtr());
}

frc2::CommandPtr RobotContainer::GetDefaultDriveCommand() {
  return FieldDriveCommand(
             &m_driveSubsystem, [this] { return m_driverController.GetLeftY(); },
             [this] { return m_driverController.GetLeftX(); }, [this] { return m_driverController.GetRightX(); })
      .ToPtr();
}
