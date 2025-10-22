// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <rev/SparkMax.h>

#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "units/length.h"
#include "units/velocity.h"
#include "wpi/sendable/Sendable.h"

class NeoKrakenModule : public wpi::Sendable {
 private:
  ctre::phoenix6::hardware::TalonFX driveMotor;
  rev::spark::SparkMax steerMotor;
  ctre::phoenix6::hardware::CANcoder encoderObject;

  double offset;
  double setpoint{};

  frc::SimpleMotorFeedforward<units::meters> ff;
  frc::PIDController driveController, steerController;

  constexpr static double kVelocityMultiplier = (1 / 6.75 / 60) * (.1016 * M_PI);
  constexpr static double kPositionMultiplier = (1 / 6.75) * (.1016 * M_PI);
  constexpr static double kCanCoderMultiplier = 2 * M_PI;

 public:
  NeoKrakenModule(int driveID, int steerID, int encoderID, double offset, const std::string& can);

  void ConfigPIDInternal();
  static void ConfigDriveMotor(ctre::phoenix6::hardware::TalonFX& target);
  static void ConfigSteerMotor(rev::spark::SparkMax& target);
  static void SetupEncoder(ctre::phoenix6::hardware::CANcoder& encoder);
  static void CurrentLimitsDrive(ctre::phoenix6::configs::TalonFXConfiguration& config);
  void SetModuleState(frc::SwerveModuleState state);
  double GetEncoderPosition();
  double GetPosition();
  frc::SwerveModuleState GetModuleState();
  frc::SwerveModulePosition GetModulePosition();
  units::meters_per_second_t GetVelocity();

  void InitSendable(wpi::SendableBuilder& builder) override;
};
