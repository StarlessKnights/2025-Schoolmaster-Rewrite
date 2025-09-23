#include "subsystems/ElevatorSubsystem.hpp"

#include "constants/Constants.h"
#include "rev/ClosedLoopSlot.h"
#include "rev/SparkBase.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLowLevel.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"
#include <cstdlib>

ElevatorSubsystem::ElevatorSubsystem() { ConfigureMotors(); }

void ElevatorSubsystem::ConfigureMotors() {
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
  onboardClosedLoop.SetReference(-position, rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                 rev::spark::kSlot0, -ElevatorSubsystemConstants::kArbitraryFeedforward,
                                 rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
}

double ElevatorSubsystem::GetPosition() { return -primaryEncoder.GetPosition(); }

bool ElevatorSubsystem::IsElevatorPIDAtSetpoint() {
  return abs(GetPosition() - currentSetpoint) < ElevatorSubsystemConstants::kAtSetpointTolerance;
}
