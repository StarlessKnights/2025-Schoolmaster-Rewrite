// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "subsystems/ElevatorSubsystem.hpp"

#include "constants/Constants.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/InstantCommand.h"
#include "rev/ClosedLoopSlot.h"
#include "rev/SparkBase.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLowLevel.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"

#include <frc2/command/RunCommand.h>

ElevatorSubsystem::ElevatorSubsystem() {
  ConfigurePrimaryMotor();
  ConfigureSecondaryMotor();
  ConfigureCoralMotor();

  SetName("ElevatorSubsystem");

  primaryEncoder.SetPosition(0.0);
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

void ElevatorSubsystem::SetSpin(const double percent) {
  primaryMotor.Set(-percent);
}

void ElevatorSubsystem::SetCoralGrabber(const double percent) {
  coralMotor.Set(percent);
}

double ElevatorSubsystem::GetPosition() const {
  return -primaryEncoder.GetPosition();
}

void ElevatorSubsystem::SetPosition(const double position) {
  currentSetpoint = position;
  onboardClosedLoop.SetReference(-position, rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
                                 rev::spark::kSlot0, -ElevatorSubsystemConstants::kArbitraryFeedforward,
                                 rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
}

void ElevatorSubsystem::ZeroEncoder() {
  primaryEncoder.SetPosition(0.0);
}

void ElevatorSubsystem::ZeroEncoder(const double customZero) {
  primaryEncoder.SetPosition(customZero);
}

void ElevatorSubsystem::StopAll() {
  SetSpin(0.0);
  SetCoralGrabber(0.0);
}

bool ElevatorSubsystem::GetIsCoralInHoldingPosition() const {
  return coralSensor.GetProximity() < ElevatorSubsystemConstants::kCoralSensorProximityThreshold;
}

bool ElevatorSubsystem::IsElevatorPIDAtSetpoint() const {
  return std::abs(currentSetpoint - GetPosition()) < ElevatorSubsystemConstants::kAtSetpointTolerance;
}

double ElevatorSubsystem::GetElevatorCurrentDraw() {
  return primaryMotor.GetOutputCurrent();
}

frc2::CommandPtr ElevatorSubsystem::MoveElevatorToPositionCommand(double position) {
  return frc2::InstantCommand([this, position] { SetPosition(position); }, {this})
      .AndThen(
          frc2::cmd::WaitUntil([this] { return IsElevatorPIDAtSetpoint(); }).WithName("MoveElevatorToPositionCommand"));
}

frc2::CommandPtr ElevatorSubsystem::ExtendToHeightAndWaitCommand(double positionSetpoint,
                                                                 const std::function<bool()>& runCoralExtruder) {
  return frc2::RunCommand(
             [this, positionSetpoint, runCoralExtruder] {
               SetPosition(positionSetpoint);
               SetCoralGrabber(runCoralExtruder() ? ElevatorSubsystemConstants::kGrabberSpeed : 0.0);
             },
             {this})
      .ToPtr()
      .WithName("ExtendToHeightAndWaitCommand")
      .FinallyDo([this] { StopAll(); });
}

frc2::CommandPtr ElevatorSubsystem::ExtendToHeightAndScoreCommand(double positionSetpoint) {
  return frc2::FunctionalCommand([]() {},
                                 [this, positionSetpoint] {
                                   SetPosition(positionSetpoint);

                                   if (IsElevatorPIDAtSetpoint()) {
                                     SetCoralGrabber(ElevatorSubsystemConstants::kGrabberSpeed);
                                   } else {
                                     SetCoralGrabber(0.0);
                                   }
                                 },
                                 [this](bool) { StopAll(); }, [] { return false; }, {this})
      .ToPtr()
      .WithName("ExtendToHeightAndScoreCommand");
}

frc2::CommandPtr ElevatorSubsystem::HPIntakeCommand() {
  return frc2::FunctionalCommand([] {},
                                 [this] {
                                   SetPosition(ElevatorSubsystemConstants::kHPEncoderPosition);
                                   SetCoralGrabber(ElevatorSubsystemConstants::kIntakeGrabberSpeed);
                                 },
                                 [this](bool) { StopAll(); }, [this] { return GetIsCoralInHoldingPosition(); }, {this})
      .ToPtr()
      .WithName("HPIntakeCommand");
}

frc2::CommandPtr ElevatorSubsystem::RetractCommand() {
  return frc2::RunCommand([this] { SetPosition(ElevatorSubsystemConstants::kDefaultEncoderPosition); }, {this})
      .ToPtr()
      .WithName("RetractCommand")
      .FinallyDo([this] { StopAll(); });
}
