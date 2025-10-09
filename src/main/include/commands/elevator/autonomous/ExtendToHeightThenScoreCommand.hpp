// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/ElevatorSubsystem.hpp"

class ExtendToHeightThenScoreCommand : public frc2::CommandHelper<frc2::Command, ExtendToHeightThenScoreCommand> {
 private:
  ElevatorSubsystem* elevator;
  double positionSetpoint;

 public:
  ExtendToHeightThenScoreCommand(ElevatorSubsystem* elevator, double positionSetpoint)
      : elevator(elevator), positionSetpoint(positionSetpoint) {
    AddRequirements(elevator);
  };

  void Initialize() override {};
  void Execute() override {
    elevator->SetPosition(positionSetpoint);

    if (elevator->IsElevatorPIDAtSetpoint()) {
      elevator->SetCoralGrabber(ElevatorSubsystemConstants::kGrabberSpeed);
    } else {
      elevator->SetCoralGrabber(0);
    }
  }
  void End(bool) override { elevator->StopAll(); }
  bool IsFinished() override { return false; }
};
