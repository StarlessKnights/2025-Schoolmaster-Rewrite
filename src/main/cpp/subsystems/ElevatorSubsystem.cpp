#include "subsystems/ElevatorSubsystem.hpp"
#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc/RobotBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/ClosedLoopSlot.h"
#include "rev/SparkBase.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLowLevel.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"
#include "units/length.h"
#include "units/voltage.h"

ElevatorSubsystem::ElevatorSubsystem() {
  ConfigurePrimaryMotor();
  ConfigureSecondaryMotor();
  ConfigureCoralMotor();

  primaryEncoder.SetPosition(0.0);

  frc::DataLogManager::Log("ElevatorSubsystem Constructor");
}

void ElevatorSubsystem::ConfigurePrimaryMotor() {
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
}

void ElevatorSubsystem::ConfigureSecondaryMotor() {
  rev::spark::SparkMaxConfig secondaryConfig{};
  secondaryConfig.Follow(primaryMotor, false);
  secondaryConfig.VoltageCompensation(RobotConstants::kNominalVoltage);
  secondaryConfig.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  secondaryConfig.SmartCurrentLimit(80);
  secondaryMotor.Configure(secondaryConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                           rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void ElevatorSubsystem::ConfigureCoralMotor() {
  rev::spark::SparkMaxConfig neoConfig{};
  neoConfig.SmartCurrentLimit(ElevatorSubsystemConstants::kNEO550CurrentLimit);
  neoConfig.Inverted(true);
  neoConfig.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  neoConfig.VoltageCompensation(RobotConstants::kNominalVoltage);
  coralMotor.Configure(neoConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                       rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void ElevatorSubsystem::SetSpin(double percent) { primaryMotor.Set(-percent); }

void ElevatorSubsystem::SetCoralGrabber(double percent) { coralMotor.Set(percent); }

double ElevatorSubsystem::GetPosition() { return -primaryEncoder.GetPosition(); }

void ElevatorSubsystem::SetPosition(double position) {
  currentSetpoint = position;

  if (frc::RobotBase::IsReal()) {
    onboardClosedLoop.SetReference(-position, rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                   rev::spark::kSlot0, -ElevatorSubsystemConstants::kArbitraryFeedforward,
                                   rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
  }
}

void ElevatorSubsystem::ZeroEncoder() { primaryEncoder.SetPosition(0.0); }

void ElevatorSubsystem::ZeroEncoder(double customZero) { primaryEncoder.SetPosition(customZero); }

void ElevatorSubsystem::StopAll() {
  SetSpin(0.0);
  SetCoralGrabber(0.0);
}

bool ElevatorSubsystem::GetIsCoralInHoldingPosition() {
  return coralSensor.GetProximity() < ElevatorSubsystemConstants::kCoralSensorProximityThreshold;
}

bool ElevatorSubsystem::IsElevatorPIDAtSetpoint() {
  return std::abs(currentSetpoint - GetPosition()) < ElevatorSubsystemConstants::kAtSetpointTolerance;
}

double ElevatorSubsystem::GetElevatorCurrentDraw() { return primaryMotor.GetOutputCurrent(); }

void ElevatorSubsystem::SimulationPeriodic() {
  double pidOutput = m_simPID.Calculate(simPosition.value(), EncoderRotationToMeters(currentSetpoint).value());

  units::volt_t appliedVoltage = units::volt_t{pidOutput};
  m_elevatorSim.SetInputVoltage(appliedVoltage);

  m_elevatorSim.Update(20_ms);

  simPosition = units::meter_t{m_elevatorSim.GetOutput(0)};

  frc::SmartDashboard::PutNumber("Sim Elevator Position", simPosition.value());
  frc::SmartDashboard::PutNumber("Setpoint", EncoderRotationToMeters(currentSetpoint).value());
}
