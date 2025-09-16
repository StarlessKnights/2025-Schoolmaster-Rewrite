// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/FieldDriveCommand.hpp"
#include "commands/PrintOutputCommand.hpp"

RobotContainer::RobotContainer() : m_driveSubsystem() {
  // Initialize all of your commands and subsystems here
  wpi::outs() << "RobotContainer initialized\n";

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_operatorController.Button(1).OnTrue(GetPrintOutputCommand("A button pressed"));
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

frc2::CommandPtr RobotContainer::GetPrintOutputCommand(const std::string &message) {
  return PrintOutputCommand(message).ToPtr();
}