// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "commands/algaegrabber/ElevatorPopUpAndAlgaeGrabberGoToPositionCommand.hpp"

#include "constants/Constants.h"

ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(
    AlgaeGrabberSubsystem* algaeGrabber, ElevatorSubsystem* elevator, const double algaeEncoderPosition)
    : algaeGrabber(algaeGrabber), elevator(elevator), algaeGrabberEncoderPosition(algaeEncoderPosition) {
  SetName("ElevatorPopUpAndAlgaeGrabberGoToPositionCommand");

  AddRequirements(algaeGrabber);
  AddRequirements(elevator);
}

void ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::Initialize() {
  const double currentPosition = elevator->GetPosition();
  constexpr double minPosition = AlgaeGrabberSubsystemsConstants::kMinimumSafeElevatorEncoderPosition;

  algaeGrabber->SetPosition(algaeGrabberEncoderPosition);
  homePosition = currentPosition < minPosition ? minPosition : currentPosition;
}

void ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::Execute() {
  elevator->SetPosition(homePosition);

  if (elevator->IsElevatorPIDAtSetpoint()) {
    algaeGrabber->SetPosition(algaeGrabberEncoderPosition);
  } else {
    algaeGrabber->SetPivotMotor(0.0);
  }
}

void ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::End(bool) {
  elevator->StopAll();
  algaeGrabber->StopAll();
}

bool ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::IsFinished() {
  return algaeGrabber->IsAlgaeGrabberPIDAtSetpoint();
}
