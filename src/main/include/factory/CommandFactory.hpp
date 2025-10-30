#pragma once

#include "frc/GenericHID.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/CommandXboxController.h"
#include "units/time.h"

namespace CommandFactory {
inline frc2::CommandPtr MakeRumbleCommand(frc2::CommandXboxController& controller, const units::second_t duration) {
  return frc2::cmd::RunEnd([&] { controller.SetRumble(frc::GenericHID::kBothRumble, 0.5); },
                           [&] { controller.SetRumble(frc::GenericHID::kBothRumble, 0); })
      .WithTimeout(duration)
      .WithName("RumbleController");
}
}  // namespace CommandFactory
