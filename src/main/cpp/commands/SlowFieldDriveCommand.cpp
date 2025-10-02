#include "units/angular_velocity.h"
#include <frc/kinematics/ChassisSpeeds.h>

#include <commands/SlowFieldDriveCommand.hpp>

void SlowFieldDriveCommand::Initialize() {};

void SlowFieldDriveCommand::Execute() {
  frc::ChassisSpeeds fieldRelativeSpeeds =
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          ySpeed() * 0.5_mps, xSpeed() * 0.5_mps,
          rotX() * units::radians_per_second_t{180_deg_per_s},
          drive->GetDriverGyroAngle());

  drive->Drive(fieldRelativeSpeeds);
}

void SlowFieldDriveCommand::End(bool interrupted) {};

bool SlowFieldDriveCommand::IsFinished() { return false; }
