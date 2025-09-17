#include "subsystems/DriveSubsystem.hpp"

#include <array>

#include "choreo/trajectory/SwerveSample.h"
#include "constants/Constants.h"
#include "frc/Timer.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/length.h"
#include "units/velocity.h"
#include "utils/TurboPoseEstimator.hpp"

DriveSubsystem::DriveSubsystem()
    : fleft(1, 2, 3, 0.0, DriveSubsystemConstants::kCanivoreName),
      fright(4, 5, 6, 0.0, DriveSubsystemConstants::kCanivoreName),
      bleft(7, 8, 9, 0.0, DriveSubsystemConstants::kCanivoreName),
      bright(11, 12, 13, 0.0, DriveSubsystemConstants::kCanivoreName),
      field(),
      estimator(GetAngle(), GetModulePositions(), frc::Pose2d()) {
  frc::SmartDashboard::PutData("Field", &field);
  m_lastTime = frc::Timer::GetFPGATimestamp();
}

void DriveSubsystem::Drive(frc::ChassisSpeeds speeds) {
  m_cmdSpeeds = speeds;
  auto states = DriveSubsystemConstants::kKinematics.ToSwerveModuleStates(speeds);
  SetModuleStates(states);
}

void DriveSubsystem::SetModuleStates(const std::array<frc::SwerveModuleState, 4>& states) {
  fleft.SetModuleState(states[0]);
  fright.SetModuleState(states[1]);
  bleft.SetModuleState(states[2]);
  bright.SetModuleState(states[3]);
}

std::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates() {
  return {fleft.GetModuleState(), fright.GetModuleState(), bleft.GetModuleState(),
          bright.GetModuleState()};
}

std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() {
  return {fleft.GetModulePosition(), fright.GetModulePosition(), bleft.GetModulePosition(),
          bright.GetModulePosition()};
}

frc::Rotation2d DriveSubsystem::GetDriverGyroAngle() { return GetAngle() - driverGyroOffset; }

void DriveSubsystem::Periodic() {
  estimator.UpdateWithOdometry(GetAngle(), GetModulePositions());
  field.SetRobotPose(estimator.getPose2D());

  frc::SmartDashboard::PutNumber("Front Left", fleft.GetEncoderPosition());
  frc::SmartDashboard::PutNumber("Front Right", fright.GetEncoderPosition());
  frc::SmartDashboard::PutNumber("Back Left", bleft.GetEncoderPosition());
  frc::SmartDashboard::PutNumber("Back Right", bright.GetEncoderPosition());
}

void DriveSubsystem::SimulationPeriodic() {
  const auto currentTime = frc::Timer::GetFPGATimestamp();
  auto dt = currentTime - m_lastTime;
  m_lastTime = currentTime;

  auto fieldRelSpeeds =
      frc::ChassisSpeeds::FromRobotRelativeSpeeds(m_cmdSpeeds, m_simPose.Rotation());

  auto newX = m_simPose.X() + fieldRelSpeeds.vx * dt;
  auto newY = m_simPose.Y() + fieldRelSpeeds.vy * dt;
  auto newTheta = frc::Rotation2d(m_simPose.Rotation().Radians() + fieldRelSpeeds.omega * dt);
  m_simPose = frc::Pose2d(newX, newY, newTheta);

  field.SetRobotPose(m_simPose);

  auto& simGyro = pigeon.GetSimState();
  simGyro.SetRawYaw(m_simPose.Rotation().Degrees());
}

void DriveSubsystem::FollowTrajectory(const choreo::SwerveSample& sample) {
  frc::Pose2d pose = estimator.getPose2D();

  units::meters_per_second_t xFeedback{xController.Calculate(pose.X().value(), sample.x.value())};
  units::meters_per_second_t yFeedback{yController.Calculate(pose.Y().value(), sample.y.value())};
  units::radians_per_second_t thetaFeedback{
      thetaController.Calculate(pose.Rotation().Radians().value(), sample.heading.value())};

  frc::ChassisSpeeds speeds{sample.vx + xFeedback, sample.vy + yFeedback,
                            sample.omega + thetaFeedback};

  Drive(speeds);
}