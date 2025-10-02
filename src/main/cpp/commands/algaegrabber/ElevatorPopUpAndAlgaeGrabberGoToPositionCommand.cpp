#include "commands/algaegrabber/ElevatorPopUpAndAlgaeGrabberGoToPositionCommand.hpp"
#include "constants/Constants.h"

ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(
    AlgaeGrabberSubsystem *algaeGrabber, ElevatorSubsystem *elevator, double algaeEncoderPosition)
    : algaeGrabber(algaeGrabber), elevator(elevator), algaeGrabberEncoderPosition(algaeEncoderPosition) {
  AddRequirements(algaeGrabber);
  AddRequirements(elevator);
}

void ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::Initialize() {
  double currentPosition = elevator->GetPosition();
  double minPosition = AlgaeGrabberSubsystemsConstants::kMinimumSafeElevatorEncoderPosition;
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

void ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::End(bool interrupted) {
  elevator->StopAll();
  algaeGrabber->StopAll();
}

bool ElevatorPopUpAndAlgaeGrabberGoToPositionCommand::IsFinished() {
  return algaeGrabber->IsAlgaeGrabberPIDAtSetpoint();
}
