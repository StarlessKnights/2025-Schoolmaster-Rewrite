#pragma once

#include "constants/Constants.h"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc2/command/SubsystemBase.h"
#include "utils/NeoKrakenModule.hpp"

class DriveSubsystem : public frc2::SubsystemBase {
  NeoKrakenModule fleft, fright, bleft, bright;
  ctre::phoenix6::hardware::Pigeon2 pigeon{DriveSubsystemConstants::kPigeonID,
                                           DriveSubsystemConstants::kCanivoreName};
  frc::Rotation2d driverGyroOffset{};

 public:
  DriveSubsystem();

  void drive(frc::ChassisSpeeds speeds);
  void setModuleStates(const std::array<frc::SwerveModuleState, 4> &states);
  frc::Rotation2d getDriverGyroAngle();
  frc::Rotation2d getAngle() { return pigeon.GetRotation2d(); }
};