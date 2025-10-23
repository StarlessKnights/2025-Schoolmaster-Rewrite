// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"

class AlgaeGrabberGoToPositionCommand final : public frc2::CommandHelper<frc2::Command, AlgaeGrabberGoToPositionCommand> {
  AlgaeGrabberSubsystem* grabber;
  double position;

 public:
  AlgaeGrabberGoToPositionCommand(AlgaeGrabberSubsystem* grabber, const double position)
      : grabber(grabber), position(position) {
    SetName("AlgaeGrabberGoToPositionCommand");

    AddRequirements(grabber);
  }

  void Initialize() override {};
  void Execute() override { grabber->SetPosition(position); }
  void End(bool) override { grabber->StopAll(); }
  bool IsFinished() override { return false; }
};
