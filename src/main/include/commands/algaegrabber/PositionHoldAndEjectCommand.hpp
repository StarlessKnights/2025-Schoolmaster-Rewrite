// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include "constants/Constants.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"

class PositionHoldAndEjectCommand
    : public frc2::CommandHelper<frc2::Command, PositionHoldAndEjectCommand> {
 private:
  AlgaeGrabberSubsystem* grabber;
  ElevatorSubsystem* elevator;
  std::function<bool()> runExtruder;

  double currentElevatorPosition = 5.0;
  double currentGrabberPosition = 0.3;

 public:
  PositionHoldAndEjectCommand(AlgaeGrabberSubsystem* grabber, ElevatorSubsystem* elevator,
                              std::function<bool()> runExtruder)
      : grabber(grabber), elevator(elevator), runExtruder(runExtruder) {
    AddRequirements(grabber);
    AddRequirements(elevator);
  };

  void Initialize() override {
    currentElevatorPosition = elevator->GetPosition();
    currentGrabberPosition = grabber->GetLinearizedPosition();
  };
  void Execute() override {
    grabber->SetPosition(currentGrabberPosition);
    elevator->SetPosition(currentElevatorPosition);
    grabber->SetSpinMotor(runExtruder() ? -AlgaeGrabberSubsystemsConstants::kIntakeMotorSpeed
                                        : 0.0);
  };
  void End(bool interrupted) override {
    grabber->StopAll();
    elevator->StopAll();
  };
  bool IsFinished() override { return false; };
};
