// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <functional>

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/DriveSubsystem.hpp"

class SlowFieldDriveCommand final : public frc2::CommandHelper<frc2::Command, SlowFieldDriveCommand> {
 private:
  std::function<double()> xSpeed, ySpeed, rotX;
  DriveSubsystem* drive;

 public:
  explicit SlowFieldDriveCommand(DriveSubsystem* drive, const std::function<double()>& ySpeed,
                                 const std::function<double()>& xSpeed,
                                 std::function<double()> rotX)
      : xSpeed(xSpeed), ySpeed(ySpeed), rotX(std::move(rotX)), drive(drive) {
    SetName("SlowFieldDriveCommand");
    AddRequirements(drive);
  }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
};
