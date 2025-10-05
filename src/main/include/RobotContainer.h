// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <functional>

#include "constants/Constants.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"
#include "subsystems/DriveSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr MakeAlgaeGrabberSequence(double elevatorPosition, std::function<bool()> runExtruder);
  frc2::CommandPtr MakeFieldDriveCommand();
  frc2::CommandPtr MakeSlowFieldDriveCommand();
  frc2::CommandPtr MakeProcessorScoreSequence(std::function<bool()> runOuttake);
  frc2::CommandPtr MakeElevatorScoreSequence(double elevatorPosition, std::function<bool()> runExtruder);
  frc2::CommandPtr MakeCancelCommand();
  frc2::CommandPtr MakeAutoCommand();  // THIS IS FOR DEVELOPMENT DONT INCLUDE IN REAL ROBOT

 private:
  frc2::CommandXboxController m_driverController{OperatorConstants::kDriverControllerPort};
  frc2::CommandXboxController m_operatorController{OperatorConstants::kOperatorControllerPort};

  DriveSubsystem m_driveSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  AlgaeGrabberSubsystem m_algaeGrabberSubsystem;

  void ConfigureBindings();
  void ConfigureElevatorBindings();
  void ConfigureAlgaeGrabberBindings();
  void ConfigureDefaultCommands();
};
