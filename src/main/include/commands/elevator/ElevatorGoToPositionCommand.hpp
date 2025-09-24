#pragma once

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/ElevatorSubsystem.hpp"

class ElevatorGoToPositionCommand : public frc2::CommandHelper<frc2::Command, ElevatorGoToPositionCommand> {
private:
  double m_position;
  ElevatorSubsystem *m_elevatorSubsystem;

public:
  explicit ElevatorGoToPositionCommand(ElevatorSubsystem *elevatorSubsystem, double position)
      : m_position(position), m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements(m_elevatorSubsystem);
  }

  void Initialize() override {}
  void Execute() override { m_elevatorSubsystem->SetPosition(m_position); }
  void End(bool interrupted) override {} // TODO: Implement this action later
  bool IsFinished() override { return false; }
};