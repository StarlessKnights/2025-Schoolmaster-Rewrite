#pragma once

#include "frc2/command/CommandHelper.h"
#include "wpi/raw_ostream.h"

class PrintOutputCommand : public frc2::CommandHelper<frc2::Command, PrintOutputCommand> {
 private:
  std::string message;

 public:
  explicit PrintOutputCommand(const std::string &message) : message(message) {}

  void Initialize() override { wpi::outs() << message << "\n"; }
  void Execute() override {}
  void End(bool interrupted) override {}
  bool IsFinished() override { return true; }
};