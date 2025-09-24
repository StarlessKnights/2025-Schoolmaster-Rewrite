#include "subsystems/ElevatorSubsystem.hpp"

#include "constants/Constants.h"
#include "frc/RobotBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/ClosedLoopSlot.h"
#include "rev/SparkBase.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLowLevel.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"
#include <cstdlib>

ElevatorSubsystem::ElevatorSubsystem() {
  frc::SmartDashboard::PutData("Elevator Mech", &elevatorMech);

  ConfigureMotors();
}

void ElevatorSubsystem::ConfigureMotors() {
  if (frc::RobotBase::IsSimulation()) {
    return;
  }

  rev::spark::SparkMaxConfig smc{};
  smc.Follow(primaryMotor, false);
  smc.VoltageCompensation(RobotConstants::kNominalVoltage);
  smc.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  smc.SmartCurrentLimit(80);
  secondaryMotor.Configure(smc, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                           rev::spark::SparkBase::PersistMode::kPersistParameters);

  rev::spark::SparkMaxConfig neoConfig{};
  neoConfig.SmartCurrentLimit(ElevatorSubsystemConstants::kNEO550CurrentLimit);
  neoConfig.Inverted(true);
  neoConfig.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  neoConfig.VoltageCompensation(RobotConstants::kNominalVoltage);
  spinGrabberMotor.Configure(neoConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                             rev::spark::SparkBase::PersistMode::kPersistParameters);

  rev::spark::SparkMaxConfig primaryConfig{};
  primaryConfig.encoder.PositionConversionFactor(1.0);
  primaryConfig.VoltageCompensation(RobotConstants::kNominalVoltage);
  primaryConfig.closedLoop.Pid(0.8, 0.0, 0.0);
  primaryConfig.SmartCurrentLimit(80);
  primaryConfig.closedLoop.maxMotion.MaxAcceleration(ElevatorSubsystemConstants::kMaxAcceleration)
      .MaxVelocity(ElevatorSubsystemConstants::kMaxVelocity)
      .AllowedClosedLoopError(0.5);
  primaryConfig.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  primaryMotor.Configure(primaryConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                         rev::spark::SparkBase::PersistMode::kPersistParameters);

  primaryEncoder.SetPosition(0.0);
}

void ElevatorSubsystem::SetPosition(double position) {
  currentSetpoint = position;

  if (!frc::RobotBase::IsSimulation()) {
    onboardClosedLoop.SetReference(-position, rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                   rev::spark::kSlot0, -ElevatorSubsystemConstants::kArbitraryFeedforward,
                                   rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
  }
}

double ElevatorSubsystem::GetPosition() {
  if (frc::RobotBase::IsSimulation()) {
    return simulatedPosition;
  }

  return -primaryEncoder.GetPosition();
}

bool ElevatorSubsystem::IsElevatorPIDAtSetpoint() {
  return abs(GetPosition() - (currentSetpoint * 0.3)) < ElevatorSubsystemConstants::kAtSetpointTolerance;
}

void ElevatorSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Elevator Position", GetPosition());
  frc::SmartDashboard::PutNumber("Elevator Setpoint", currentSetpoint);
  frc::SmartDashboard::PutBoolean("Elevator At Setpoint", IsElevatorPIDAtSetpoint());
}

void ElevatorSubsystem::SimulationPeriodic() {
  double kSimSpeed = 0.06;
  if (abs(simulatedPosition - (currentSetpoint * 0.3)) > 0.01) {
    simulatedPosition += kSimSpeed * ((currentSetpoint * 0.3) - simulatedPosition);
  } else {
    simulatedPosition = currentSetpoint * 0.3;
  }

  m_elevator->SetLength(simulatedPosition);
  frc::SmartDashboard::PutNumber("Simulated Elevator Position", simulatedPosition);
}