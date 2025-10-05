// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "subsystems/AlgaeGrabberSubsystem.hpp"

#include "constants/Constants.h"
#include "rev/SparkBase.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"

AlgaeGrabberSubsystem::AlgaeGrabberSubsystem() {
  ConfigurePivotMotor();
  ConfigureSpinMotor();

  SetName("AlgaeGrabberSubsystem");

  controller.SetTolerance(0.1);
}

void AlgaeGrabberSubsystem::ConfigurePivotMotor() {
  rev::spark::SparkMaxConfig config{};
  config.VoltageCompensation(RobotConstants::kNominalVoltage);
  config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);

  pivotMotor.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                       rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void AlgaeGrabberSubsystem::ConfigureSpinMotor() {
  rev::spark::SparkMaxConfig config{};
  config.VoltageCompensation(RobotConstants::kNominalVoltage);
  config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  config.OpenLoopRampRate(0.25);

  spinMotor.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                      rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void AlgaeGrabberSubsystem::SetSpinMotor(double percent) {
  spinMotor.Set(-percent);
}

void AlgaeGrabberSubsystem::SetPivotMotor(double percent) {
  pivotMotor.Set(percent);
}

double AlgaeGrabberSubsystem::LinearizeEncoderOutput(double currentPosition) {
  if (currentPosition > 0.5) {
    return currentPosition - 1.0;
  }

  return currentPosition;
}

void AlgaeGrabberSubsystem::SetPosition(double position) {
  double currentPosition = LinearizeEncoderOutput(GetPosition());
  double speed = controller.Calculate(currentPosition, position);
  SetPivotMotor(speed);
}

void AlgaeGrabberSubsystem::StopAll() {
  spinMotor.Set(0.0);
  pivotMotor.Set(0.0);
}

double AlgaeGrabberSubsystem::GetPosition() {
  return thruBore.Get();
}

double AlgaeGrabberSubsystem::GetLinearizedPosition() {
  return LinearizeEncoderOutput(GetPosition());
}

double AlgaeGrabberSubsystem::GetSpinMotorCurrentDraw() {
  return spinMotor.GetOutputCurrent();
}

bool AlgaeGrabberSubsystem::IsAlgaeGrabberPIDAtSetpoint() {
  return controller.AtSetpoint();
}
