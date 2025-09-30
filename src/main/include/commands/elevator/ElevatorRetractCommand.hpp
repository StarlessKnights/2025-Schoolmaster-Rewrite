#pragma once

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/ElevatorSubsystem.hpp"

class ElevatorRetractCommand : public frc2::CommandHelper<frc2::Command, ElevatorRetractCommand> {
private:
  ElevatorSubsystem *elevator;

public:
  explicit ElevatorRetractCommand(ElevatorSubsystem *elevator) : elevator(elevator) { AddRequirements(elevator); };

  void Initialize() override;
  void Execute() override { elevator->SetPosition(ElevatorSubsystemConstants::kDefaultEncoderPosition); };
  void End(bool interrupted) override { elevator->StopAll(); };
  bool IsFinished() override { return false; };
};
