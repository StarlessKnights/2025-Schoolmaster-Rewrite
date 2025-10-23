// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "utils/NeoKrakenModule.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <string>

#include "constants/Constants.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "frc/controller/PIDController.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "frc/geometry/Rotation2d.h"
#include "rev/SparkBase.h"
#include "rev/SparkMax.h"
#include "rev/config/SparkBaseConfig.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/length.h"
#include "wpi/sendable/SendableBuilder.h"

NeoKrakenModule::NeoKrakenModule(const int driveID, const int steerID, const int encoderID, const double offset,
                                 const std::string& can)
    : driveMotor(driveID, can),
      steerMotor(steerID, rev::spark::SparkLowLevel::MotorType::kBrushless),
      encoderObject(encoderID, can),
      offset(offset),
      ff(0_V, 0_V / 1_mps),
      driveController(0.0, 0.0, 0.0),
      steerController(0.0, 0.0, 0.0) {
  SetupEncoder(encoderObject);
  ConfigDriveMotor(driveMotor);
  ConfigSteerMotor(steerMotor);
  ConfigPIDInternal();
}

void NeoKrakenModule::SetupEncoder(ctre::phoenix6::hardware::CANcoder& encoder) {
  ctre::phoenix6::configs::CANcoderConfigurator& configPls = encoder.GetConfigurator();
  ctre::phoenix6::configs::CANcoderConfiguration config{};
  config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = units::turn_t{0.5};

  configPls.Apply(config);
}

void NeoKrakenModule::ConfigPIDInternal() {
  this->ff = frc::SimpleMotorFeedforward<units::meters>{0.015_V, 0.212_V / 1_mps};

  this->driveController = frc::PIDController(0.01, 0.0, 0.0);
  this->steerController = frc::PIDController(0.3, 0.0, 0.0);

  this->steerController.EnableContinuousInput(-M_PI, M_PI);
}

void NeoKrakenModule::ConfigDriveMotor(ctre::phoenix6::hardware::TalonFX& target) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  CurrentLimitsDrive(config);

  target.GetConfigurator().Apply(config);
}

void NeoKrakenModule::ConfigSteerMotor(rev::spark::SparkMax& target) {
  rev::spark::SparkBaseConfig config;
  config.Inverted(false);
  config.VoltageCompensation(NeoKrakenModuleConstants::kNominalVoltage);
  config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  config.SmartCurrentLimit(80);
  target.Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                   rev::spark::SparkBase::PersistMode::kPersistParameters);
}

// Configure Kraken Motor's CurrentLimits
void NeoKrakenModule::CurrentLimitsDrive(ctre::phoenix6::configs::TalonFXConfiguration& config) {
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimitEnable = true;

  config.CurrentLimits.SupplyCurrentLimit = units::ampere_t{70};
  config.CurrentLimits.StatorCurrentLimit = units::ampere_t{120};
}

void NeoKrakenModule::SetModuleState(frc::SwerveModuleState state) {
  const double currentMeasurement = GetEncoderPosition();
  const frc::Rotation2d currentAngle{units::radian_t{currentMeasurement}};
  state.Optimize(currentAngle);

  const units::meters_per_second_t speed = state.speed;
  const units::radian_t angle = state.angle.Radians();
  setpoint = angle.value();

  const double drivePercent = driveController.Calculate(GetVelocity().value(), speed.value());
  const double steerPercent = steerController.Calculate(currentMeasurement, angle.value());

  driveMotor.Set(drivePercent + ff.Calculate(speed).value());
  steerMotor.Set(-steerPercent);
}

void NeoKrakenModule::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("SwerveModule");
  builder.AddDoubleProperty("Drive Velocity (m/s)", [this]() { return GetVelocity().value(); }, nullptr);
  builder.AddDoubleProperty("Drive Position (m)", [this]() { return GetPosition(); }, nullptr);
  builder.AddDoubleProperty("Steer Angle (rad)", [this]() { return GetEncoderPosition(); }, nullptr);
  builder.AddDoubleProperty("Steer Setpoint (rad)", [this]() { return setpoint; }, nullptr);
}

double NeoKrakenModule::GetEncoderPosition() {
  const ctre::phoenix6::StatusSignal<units::turn_t> angle = encoderObject.GetAbsolutePosition();
  return (angle.GetValueAsDouble() * kCanCoderMultiplier) - offset;
}

double NeoKrakenModule::GetPosition() {
  const ctre::phoenix6::StatusSignal<units::turn_t> position = driveMotor.GetPosition();
  return position.GetValue().value() * kPositionMultiplier;
}

frc::SwerveModuleState NeoKrakenModule::GetModuleState() {
  return {GetVelocity(), frc::Rotation2d{units::radian_t{GetEncoderPosition()}}};
}

frc::SwerveModulePosition NeoKrakenModule::GetModulePosition() {
  return {units::meter_t{GetPosition()}, units::radian_t{M_PI + GetEncoderPosition()}};
}

units::meters_per_second_t NeoKrakenModule::GetVelocity() {
  const ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocity = driveMotor.GetVelocity();
  return units::meters_per_second_t{velocity.GetValue().value() * kVelocityMultiplier};
}
