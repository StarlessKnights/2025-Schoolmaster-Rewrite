#include <frc2/command/Commands.h>

#include <utils/AutonomousChooser.hpp>

#include "frc/smartdashboard/SmartDashboard.h"
#include "pathplanner/lib/auto/AutoBuilder.h"

AutonomousChooser::AutonomousChooser() {
  m_chooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
}

frc2::Command* AutonomousChooser::GetAutonomousCommand() { return m_chooser.GetSelected(); }