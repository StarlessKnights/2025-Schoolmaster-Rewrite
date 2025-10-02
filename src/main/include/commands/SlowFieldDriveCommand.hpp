#pragma once

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/DriveSubsystem.hpp"
#include <functional>

class FieldDriveCommand : public frc2::CommandHelper<frc2::Command, FieldDriveCommand> {
private:
  std::function<double()> xSpeed, ySpeed, rotX;
  DriveSubsystem *drive;

public:
  explicit FieldDriveCommand(DriveSubsystem *drive, std::function<double()> ySpeed, std::function<double()> xSpeed, std::function<double()> rotX)
      : xSpeed(xSpeed), ySpeed(ySpeed), rotX(rotX), drive(drive) {
    AddRequirements(drive);
  }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
};