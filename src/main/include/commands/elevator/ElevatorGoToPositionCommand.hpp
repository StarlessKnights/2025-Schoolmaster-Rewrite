// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <functional>
#include <utility>

#include "constants/Constants.h"
#include "frc/DataLogManager.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/ElevatorSubsystem.hpp"

class ElevatorGoToPositionCommand final : public frc2::CommandHelper<frc2::Command, ElevatorGoToPositionCommand> {
 private:
  ElevatorSubsystem* elevator;
  double positionSetpoint;
  std::function<bool()> runCoralExtruder;

 public:
  explicit ElevatorGoToPositionCommand(ElevatorSubsystem* elevator, std::function<bool()> runExtruder,
                                       const double positionSetpoint)
      : elevator(elevator), positionSetpoint(positionSetpoint), runCoralExtruder(std::move(runExtruder)) {
    SetName("ElevatorGoToPositionCommand");

    AddRequirements(elevator);
  }

  void Initialize() override {
    frc::DataLogManager::Log("ElevatorGoToPositionCommand to " + std::to_string(positionSetpoint) + " started");
  };
  void Execute() override {
    const bool runExtruder = runCoralExtruder();

    elevator->SetPosition(positionSetpoint);
    elevator->SetCoralGrabber(runExtruder ? ElevatorSubsystemConstants::kGrabberSpeed : 0.0);
  };
  void End(bool) override { elevator->StopAll(); };
  bool IsFinished() override { return false; };
};
