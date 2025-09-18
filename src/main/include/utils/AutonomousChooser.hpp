#pragma once

#include "frc/smartdashboard/SendableChooser.h"
#include "frc2/command/Command.h"

class AutonomousChooser {
 public:
  AutonomousChooser();

  frc2::Command* GetAutonomousCommand();

 private:
  frc::SendableChooser<frc2::Command*> m_chooser;
};