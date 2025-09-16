#pragma once

#include "frc/kinematics/SwerveModuleState.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <rev/SparkMax.h>
#include <string>

class NeoKrakenModule {
private:
  ctre::phoenix6::hardware::TalonFX driveMotor;
  rev::spark::SparkMax steerMotor;
  ctre::phoenix6::hardware::CANcoder encoderObject;

  double offset, setPoint;

  frc::SimpleMotorFeedforward<units::meters> ff;
  frc::PIDController driveController, steerController;

  constexpr static double kVelocityMultiplier = (1 / 6.75 / 60) * (.1016 * M_PI); // 6.75:1 reduction, m/s per RPM
  constexpr static double kPositionMultiplier = (1 / 6.75) * (.1016 * M_PI);      // 6.75:1 reduction, m per rotation
  constexpr static double kCanCoderMultiplier = 2 * M_PI;                         // degrees per tick

public:
  NeoKrakenModule(int driveID, int steerID, int encoderID, double offset, const std::string &can);

  void configPIDInternal();
  void configDriveMotor(ctre::phoenix6::hardware::TalonFX &target);
  void configSteerMotor(rev::spark::SparkMax &target);
  void setupEncoder(ctre::phoenix6::hardware::CANcoder &target);
  void currentLimitsDrive(ctre::phoenix6::configs::TalonFXConfiguration &config);
  void setModuleState(frc::SwerveModuleState state);
  double getEncoderPosition();
  units::turns_per_second_t getVelocity();
};