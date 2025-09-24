#pragma once

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/ElevatorSubsystem.hpp"

class ElevatorRetractCommand : public frc2::CommandHelper<frc2::Command, ElevatorRetractCommand> {
private:
  ElevatorSubsystem *m_elevatorSubsystem;

public:
  explicit ElevatorRetractCommand(ElevatorSubsystem *elevatorSubsystem) : m_elevatorSubsystem(elevatorSubsystem) {
    AddRequirements(m_elevatorSubsystem);
  }

  void Initialize() override {}
  void Execute() override { m_elevatorSubsystem->SetPosition(ElevatorSubsystemConstants::kDefaultEncoderPosition); }
  void End(bool interrupted) override {} // TODO: Implement this action later
  bool IsFinished() override { return false; }
};