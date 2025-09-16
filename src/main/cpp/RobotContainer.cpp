// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/FieldDriveCommand.hpp"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Commands.h"

RobotContainer::RobotContainer() : m_driveSubsystem() {
  ConfigureBindings();
  ConfigureDefaultCommands();
}

void RobotContainer::ConfigureBindings() {
  m_driverController.Button(1).OnTrue(
      frc2::cmd::RunOnce([] { frc::SmartDashboard::PutString("Hello", "World"); }, {}));
}

void RobotContainer::ConfigureDefaultCommands() {
  m_driveSubsystem.SetDefaultCommand(GetDefaultDriveCommand());
}

frc2::CommandPtr RobotContainer::GetDefaultDriveCommand() {
  return FieldDriveCommand(
             &m_driveSubsystem, [this] { return m_driverController.GetLeftY(); },
             [this] { return m_driverController.GetLeftX(); },
             [this] { return m_driverController.GetRightX(); })
      .ToPtr();
}