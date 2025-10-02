#pragma once

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"

class AlgaeGrabberAndElevatorPositionAndIntakeCommand
    : public frc2::CommandHelper<
          frc2::Command, AlgaeGrabberAndElevatorPositionAndIntakeCommand> {
private:
  ElevatorSubsystem *elevator;
  AlgaeGrabberSubsystem *grabber;

  double elevatorPosition;
  double grabberPosition;

public:
  AlgaeGrabberAndElevatorPositionAndIntakeCommand(
      ElevatorSubsystem *elevator, AlgaeGrabberSubsystem *grabber,
      double elevatorPosition, double grabberPosition)
      : elevator(elevator), grabber(grabber),
        elevatorPosition(elevatorPosition), grabberPosition(grabberPosition) {
    AddRequirements(elevator);
    AddRequirements(grabber);
  };

  void Initialize() override {};
  void Execute() override {
    elevator->SetPosition(elevatorPosition);

    if (elevator->GetPosition() >
        AlgaeGrabberSubsystemsConstants::kMinimumSafeElevatorEncoderPosition) {
      grabber->SetPosition(grabberPosition);
    } else {
      grabber->SetPivotMotor(0.0);
    }

    if (grabber->IsAlgaeGrabberPIDAtSetpoint() &&
        elevator->IsElevatorPIDAtSetpoint()) {
      grabber->SetSpinMotor(AlgaeGrabberSubsystemsConstants::kIntakeMotorSpeed);
    }
  }
  void End(bool interrupted) override {
    elevator->StopAll();
    grabber->StopAll();
  };
  bool IsFinished() override {
    return grabber->GetSpinMotorCurrentDraw() >
           AlgaeGrabberSubsystemsConstants::kIntakeCurrentDraw;
  }
};
