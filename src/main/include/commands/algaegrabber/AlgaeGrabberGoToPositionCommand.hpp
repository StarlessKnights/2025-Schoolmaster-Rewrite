// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"

class AlgaeGrabberGoToPositionCommand : public frc2::CommandHelper<frc2::Command, AlgaeGrabberGoToPositionCommand> {
 private:
  AlgaeGrabberSubsystem* grabber;
  double position;

 public:
  AlgaeGrabberGoToPositionCommand(AlgaeGrabberSubsystem* grabber, double position)
      : grabber(grabber), position(position) {
    AddRequirements(grabber);
  }

  void Initialize() override {};
  void Execute() override { grabber->SetPosition(position); }
  void End(bool) override { grabber->StopAll(); }
  bool IsFinished() override { return false; }
};
