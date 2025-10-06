// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc2/command/CommandScheduler.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "utils/AutoAlignCommandFactory.hpp"

Robot::Robot() {
  frc::DataLogManager::Start();
  AutoAlignCommandFactory::Initialize();
  frc::DataLogManager::Log("Robot initialized");
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  frc::SmartDashboard::PutData(&frc2::CommandScheduler::GetInstance());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
