#pragma once

#include "constants/Constants.h"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "frc/geometry/Rotation2d.h"
#include "frc2/command/SubsystemBase.h"
#include "utils/NeoKrakenModule.hpp"

class DriveSubsystem : public frc2::SubsystemBase {
  NeoKrakenModule fleft, fright, bleft, bright;
  ctre::phoenix6::hardware::Pigeon2 pigeon{DriveSubsystemConstants::kPigeonID, DriveSubsystemConstants::kCanivoreName};
  frc::Rotation2d driverGyroOffset{};

public:
  DriveSubsystem();
};