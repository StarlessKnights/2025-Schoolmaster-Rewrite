// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "subsystems/AlgaeGrabberSubsystem.hpp"

#include "constants/Constants.h"
#include "rev/SparkBase.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>

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

void AlgaeGrabberSubsystem::SetSpinMotor(const double percent) {
  spinMotor.Set(-percent);
}

void AlgaeGrabberSubsystem::SetPivotMotor(const double percent) {
  pivotMotor.Set(percent);
}

double AlgaeGrabberSubsystem::LinearizeEncoderOutput(const double currentPosition) {
  if (currentPosition > 0.5) {
    return currentPosition - 1.0;
  }

  return currentPosition;
}

void AlgaeGrabberSubsystem::SetPosition(const double position) {
  const double currentPosition = LinearizeEncoderOutput(GetPosition());
  const double speed = controller.Calculate(currentPosition, position);
  SetPivotMotor(speed);
}

void AlgaeGrabberSubsystem::StopAll() {
  spinMotor.Set(0.0);
  pivotMotor.Set(0.0);
}

double AlgaeGrabberSubsystem::GetPosition() const {
  return thruBore.Get();
}

double AlgaeGrabberSubsystem::GetLinearizedPosition() const {
  return LinearizeEncoderOutput(GetPosition());
}

double AlgaeGrabberSubsystem::GetSpinMotorCurrentDraw() {
  return spinMotor.GetOutputCurrent();
}

bool AlgaeGrabberSubsystem::IsAlgaeGrabberPIDAtSetpoint() const {
  return controller.AtSetpoint();
}

frc2::CommandPtr AlgaeGrabberSubsystem::PositionAndIntakeCommand(ElevatorSubsystem* elevator,
                                                                 const double elevatorPosition,
                                                                 const double grabberPosition) {
  return frc2::RunCommand(
             [this, elevator, elevatorPosition, grabberPosition] {
               elevator->SetPosition(elevatorPosition);

               if (elevator->GetPosition() > AlgaeGrabberSubsystemsConstants::kMinimumSafeElevatorEncoderPosition)
                 SetPosition(grabberPosition);
               else
                 SetPivotMotor(0.0);

               if (IsAlgaeGrabberPIDAtSetpoint() && elevator->IsElevatorPIDAtSetpoint())
                 SetSpinMotor(AlgaeGrabberSubsystemsConstants::kIntakeMotorSpeed);
             },
             {this, elevator})
      .Until([this] { return GetSpinMotorCurrentDraw() > AlgaeGrabberSubsystemsConstants::kIntakeCurrentDraw; })
      .FinallyDo([this, elevator](bool) {
        elevator->StopAll();
        StopAll();
      })
      .WithName("PositionAndIntakeAlgae");
}

frc2::CommandPtr AlgaeGrabberSubsystem::GoToPositionCommand(const double position) {
  return frc2::RunCommand([this, position] { SetPosition(position); }, {this})
      .FinallyDo([this](bool) { StopAll(); })
      .WithName("GoToPosition");
}

frc2::CommandPtr AlgaeGrabberSubsystem::PositionHoldAndEjectCommand(ElevatorSubsystem* elevator,
                                                                    const std::function<bool()>& runExtruder) {
  double currentElevatorPosition = 5.0;
  double currentGrabberPosition = 0.3;

  return frc2::FunctionalCommand(
             [this, &currentElevatorPosition, &currentGrabberPosition, elevator] {
               currentElevatorPosition = elevator->GetPosition();
               currentGrabberPosition = GetLinearizedPosition();
             },
             [this, elevator, &currentGrabberPosition, &currentElevatorPosition, runExtruder] {
               SetPosition(currentGrabberPosition);
               elevator->SetPosition(currentElevatorPosition);
               SetSpinMotor(runExtruder() ? -AlgaeGrabberSubsystemsConstants::kIntakeMotorSpeed : 0.0);
             },
             [=, this](bool) {
               StopAll();
               elevator->StopAll();
             },
             [] { return false; }, {this, elevator})
      .ToPtr()
      .WithName("PositionHoldAndEject");
}

frc2::CommandPtr AlgaeGrabberSubsystem::UnsafeProcessorScoreCommand(ElevatorSubsystem* elevator,
                                                                    const std::function<bool()>& runExtruder) {
  return frc2::FunctionalCommand(
             []() {},
             [this, elevator, runExtruder] {
               elevator->SetPosition(ElevatorSubsystemConstants::kProcessorScorePosition);
               SetPosition(AlgaeGrabberSubsystemsConstants::kProcessorScoringEncoderPosition);

               SetSpinMotor(runExtruder() ? -AlgaeGrabberSubsystemsConstants::kIntakeMotorSpeed : 0.0);
             },
             [this, elevator](bool) {
               elevator->StopAll();
               StopAll();
             },
             [this]() { return GetSpinMotorCurrentDraw() > AlgaeGrabberSubsystemsConstants::kIntakeCurrentDraw; })
      .ToPtr()
      .WithName("UnsafeProcessorScore");
}
