// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <functional>
#include <utility>

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"

// ! If grabber is not in the right position, this will damage the mechanism

class UnsafeProcessorScoreCommand final : public frc2::CommandHelper<frc2::Command, UnsafeProcessorScoreCommand> {
  AlgaeGrabberSubsystem* grabber;
  ElevatorSubsystem* elevator;
  std::function<bool()> runExtruder;

 public:
  UnsafeProcessorScoreCommand(AlgaeGrabberSubsystem* grabber, ElevatorSubsystem* elevator,
                              std::function<bool()> runExtruder)
      : grabber(grabber), elevator(elevator), runExtruder(std::move(runExtruder)) {
    SetName("UnsafeProcessorScoreCommand");
  }

  void Initialize() override {};
  void Execute() override {
    elevator->SetPosition(ElevatorSubsystemConstants::kProcessorScorePosition);
    grabber->SetPosition(AlgaeGrabberSubsystemsConstants::kProcessorScoringEncoderPosition);

    grabber->SetSpinMotor(runExtruder() ? -AlgaeGrabberSubsystemsConstants::kIntakeMotorSpeed : 0.0);
  }
  void End(bool) override {
    elevator->StopAll();
    grabber->StopAll();
  }
  bool IsFinished() override {
    return grabber->GetSpinMotorCurrentDraw() > AlgaeGrabberSubsystemsConstants::kIntakeCurrentDraw;
  }
};
