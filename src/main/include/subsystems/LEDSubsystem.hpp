// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <ranges>

#include "constants/Constants.h"
#include "frc/AddressableLED.h"
#include "frc/LEDPattern.h"
#include "frc/util/Color.h"
#include "frc2/command/SubsystemBase.h"

#include <frc2/command/RunCommand.h>

class LEDSubsystem final : public frc2::SubsystemBase {
  frc::AddressableLED addressableLED{LEDSubsystemConstants::kLEDPort};
  std::array<frc::AddressableLED::LEDData, LEDSubsystemConstants::kBufferLength> ledBuffer;

  decltype(std::ranges::take_view(ledBuffer, 9)) viewRight;
  decltype(std::ranges::subrange(ledBuffer.begin() + 14,
                                 ledBuffer.begin() + LEDSubsystemConstants::kBufferLength)) viewLeft;
  decltype(std::ranges::subrange(ledBuffer.begin() + 15, ledBuffer.begin() + 17)) viewCenter;

  frc::LEDPattern onPattern = frc::LEDPattern::Solid(frc::Color::kGreen).Blink(LEDSubsystemConstants::kBlinkOnTime);
  frc::LEDPattern offPattern = frc::LEDPattern::Solid(frc::Color::kRed);

 public:
  LEDSubsystem()
      : viewRight(std::ranges::views::take(ledBuffer, 9)),
        viewLeft(std::ranges::subrange(ledBuffer.begin() + 14, ledBuffer.end())),
        viewCenter(std::ranges::subrange(ledBuffer.begin() + 9, ledBuffer.begin() + 11)) {
    SetName("LEDSubsystem");

    addressableLED.SetLength(static_cast<int>(ledBuffer.max_size()));
    addressableLED.SetData(ledBuffer);
    addressableLED.Start();
  }

  void LeftOn() {
    SetLeft(onPattern);
    SetRight(offPattern);
    UpdateBuffer();
  }

  void RightOn() {
    SetRight(onPattern);
    SetLeft(offPattern);
    UpdateBuffer();
  }

  void SetMiddle(const frc::LEDPattern& pattern) { pattern.ApplyTo(viewCenter); }
  void SetLeft(const frc::LEDPattern& pattern) { pattern.ApplyTo(viewLeft); }
  void SetRight(const frc::LEDPattern& pattern) { pattern.ApplyTo(viewRight); }

  void UpdateBuffer() { addressableLED.SetData(ledBuffer); }

  void StopAll() {
    SetMiddle(frc::LEDPattern::Off());
    SetRight(frc::LEDPattern::Off());
    SetLeft(frc::LEDPattern::Off());
  }

  frc2::CommandPtr IndicateSideCommand(const std::function<bool()>& scoringOnLeft,
                                       const std::function<bool()>& isManuallyOverridden) {
    return frc2::RunCommand([this, scoringOnLeft, isManuallyOverridden] {
             if (isManuallyOverridden()) {
               const frc::LEDPattern p = LEDSubsystemConstants::kManualModeOn;
               SetMiddle(p);
               SetLeft(p);
               SetRight(p);
               UpdateBuffer();
             }

             if (scoringOnLeft()) {
               LeftOn();
             } else {
               RightOn();
             }
           }, {this})
        .ToPtr()
        .WithName("IndicateSideCommand");
  }
};
