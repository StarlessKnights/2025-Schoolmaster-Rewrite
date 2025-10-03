// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "constants/Constants.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/AlgaeGrabberSubsystem.hpp"
#include "subsystems/ElevatorSubsystem.hpp"

class ElevatorPopUpAndAlgaeGrabberGoToPositionCommand
    : public frc2::CommandHelper<frc2::Command, ElevatorPopUpAndAlgaeGrabberGoToPositionCommand> {
  AlgaeGrabberSubsystem* algaeGrabber;
  ElevatorSubsystem* elevator;
  double algaeGrabberEncoderPosition;
  double homePosition = AlgaeGrabberSubsystemsConstants::kMinimumSafeElevatorEncoderPosition;

 public:
  ElevatorPopUpAndAlgaeGrabberGoToPositionCommand(AlgaeGrabberSubsystem* algaeGrabber,
                                                  ElevatorSubsystem* elevator,
                                                  double algaeEncoderPosition);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
};
