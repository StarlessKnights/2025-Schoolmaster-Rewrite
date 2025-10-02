#pragma once

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"
#include <functional>

class UnsafeProcessorScoreCommand
    : public frc2::CommandHelper<frc2::Command, UnsafeProcessorScoreCommand> {
private:
  AlgaeGrabberSubsystem *grabber;
  ElevatorSubsystem *elevator;
  std::function<bool()> runExtruder;

public:
  UnsafeProcessorScoreCommand(AlgaeGrabberSubsystem *grabber,
                              ElevatorSubsystem *elevator,
                              std::function<bool()> runExtruder)
      : grabber(grabber), elevator(elevator), runExtruder(runExtruder) {};

  void Initialize() override {};
  void Execute() override {
    elevator->SetPosition(ElevatorSubsystemConstants::kProcessorScorePosition);
    grabber->SetPosition(
        AlgaeGrabberSubsystemsConstants::kProcessorScoringEncoderPosition);

    grabber->SetSpinMotor(
        runExtruder() ? -AlgaeGrabberSubsystemsConstants::kIntakeMotorSpeed
                      : 0.0);
  }
  void End(bool interrupted) override {
    elevator->StopAll();
    grabber->StopAll();
  }
  bool IsFinished() override {
    return grabber->GetSpinMotorCurrentDraw() >
           AlgaeGrabberSubsystemsConstants::kIntakeCurrentDraw;
  }
};
