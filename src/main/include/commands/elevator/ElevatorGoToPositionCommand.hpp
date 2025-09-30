#pragma once

#include "frc/DataLogManager.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/ElevatorSubsystem.hpp"

class ElevatorGoToPositionCommand : public frc2::CommandHelper<frc2::Command, ElevatorGoToPositionCommand> {
private:
  ElevatorSubsystem *elevator;
  double positionSetpoint;

public:
  explicit ElevatorGoToPositionCommand(ElevatorSubsystem *elevator, double positionSetpoint)
      : elevator(elevator), positionSetpoint(positionSetpoint) {
    AddRequirements(elevator);
  };

  void Initialize() override {
    frc::DataLogManager::Log("ElevatorGoToPositionCommand to " + std::to_string(positionSetpoint) + " started");
  };
  void Execute() override { elevator->SetPosition(positionSetpoint); };
  void End(bool interrupted) override { elevator->StopAll(); };
  bool IsFinished() override { return false; };
};
