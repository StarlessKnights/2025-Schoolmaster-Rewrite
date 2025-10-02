#pragma once

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/DriveSubsystem.hpp"
#include <functional>

class SlowFieldDriveCommand
    : public frc2::CommandHelper<frc2::Command, SlowFieldDriveCommand> {
private:
  std::function<double()> xSpeed, ySpeed, rotX;
  DriveSubsystem *drive;

public:
  explicit SlowFieldDriveCommand(DriveSubsystem *drive,
                                 std::function<double()> ySpeed,
                                 std::function<double()> xSpeed,
                                 std::function<double()> rotX)
      : xSpeed(xSpeed), ySpeed(ySpeed), rotX(rotX), drive(drive) {
    AddRequirements(drive);
  }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
};
