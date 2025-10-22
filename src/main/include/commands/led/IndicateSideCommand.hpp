// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <functional>

#include "constants/Constants.h"
#include "frc/LEDPattern.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/LEDSubsystem.hpp"

class IndicateSideCommand final : public frc2::CommandHelper<frc2::Command, IndicateSideCommand> {
 private:
  LEDSubsystem* led;
  std::function<bool()> scoringOnLeft;
  std::function<bool()> isManuallyOverridden;

 public:
  explicit IndicateSideCommand(LEDSubsystem* led, const std::function<bool()>& scoringOnLeft,
                               std::function<bool()> isManuallyOverridden)
      : led(led), scoringOnLeft(scoringOnLeft), isManuallyOverridden(std::move(isManuallyOverridden)) {
    SetName("IndicateSideCommand");
    AddRequirements(led);
  }

  void Initialize() override {};
  void Execute() override {
    if (isManuallyOverridden()) {
      const frc::LEDPattern p = LEDSubsystemConstants::kManualModeOn;
      led->SetMiddle(p);
      led->SetLeft(p);
      led->SetRight(p);
      led->UpdateBuffer();
    }

    if (scoringOnLeft()) {
      led->LeftOn();
    } else {
      led->RightOn();
    }
  }
  void End(bool) override {};
  bool IsFinished() override { return false; }
};
