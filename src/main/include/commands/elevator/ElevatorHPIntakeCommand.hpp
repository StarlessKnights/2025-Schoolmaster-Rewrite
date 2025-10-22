// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/ElevatorSubsystem.hpp"

class ElevatorHPIntakeCommand final : public frc2::CommandHelper<frc2::Command, ElevatorHPIntakeCommand> {
  ElevatorSubsystem* elevator;

 public:
  explicit ElevatorHPIntakeCommand(ElevatorSubsystem* elevator) : elevator(elevator) {
    SetName("ElevatorHPIntakeCommand");

    AddRequirements(elevator);
  }

  void Initialize() override {}
  void Execute() override {
    elevator->SetPosition(ElevatorSubsystemConstants::kHPEncoderPosition);
    elevator->SetCoralGrabber(ElevatorSubsystemConstants::kIntakeGrabberSpeed);
  }
  void End(bool) override { elevator->StopAll(); }
  bool IsFinished() override { return elevator->GetIsCoralInHoldingPosition(); }
};
