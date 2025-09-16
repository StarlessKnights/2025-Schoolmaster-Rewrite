#include <frc/smartdashboard/SmartDashboard.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <string>
#include <utils/NeoKrakenModule.hpp>

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

NeoKrakenModule::NeoKrakenModule(int driveID, int steerID, int encoderID, double offset,
                                 const std::string &can)
    : driveMotor(driveID, can),
      steerMotor(steerID, rev::spark::SparkLowLevel::MotorType::kBrushless),
      encoderObject(encoderID, can),
      offset(offset),
      ff(0_V, 0_V / 1_mps),
      driveController(0.0, 0.0, 0.0),
      steerController(0.0, 0.0, 0.0) {
  setupEncoder(encoderObject);
  configDriveMotor(driveMotor);
  configSteerMotor(steerMotor);
  configPIDInternal();
}

void NeoKrakenModule::setupEncoder(ctre::phoenix6::hardware::CANcoder &encoder) {
  ctre::phoenix6::configs::CANcoderConfigurator &configPls = encoder.GetConfigurator();
  ctre::phoenix6::configs::CANcoderConfiguration config{};
  config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = units::turn_t{0.5};

  configPls.Apply(config);
}

void NeoKrakenModule::configPIDInternal() {
  this->ff = frc::SimpleMotorFeedforward<units::meters>{0.015_V, 0.212_V / 1_mps};

  this->driveController = frc::PIDController(0.01, 0.0, 0.0);
  this->steerController = frc::PIDController(0.3, 0.0, 0.0);

  this->steerController.EnableContinuousInput(-M_PI, M_PI);
}

void NeoKrakenModule::configDriveMotor(ctre::phoenix6::hardware::TalonFX &target) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  currentLimitsDrive(config);

  target.GetConfigurator().Apply(config);
}

void NeoKrakenModule::configSteerMotor(rev::spark::SparkMax &target) {
  rev::spark::SparkBaseConfig config;
  config.Inverted(false);
  config.VoltageCompensation(NeoKrakenModuleConstants::kNominalVoltage);
  config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  config.SmartCurrentLimit(80);
  target.Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                   rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void NeoKrakenModule::currentLimitsDrive(ctre::phoenix6::configs::TalonFXConfiguration &config) {
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimitEnable = true;

  config.CurrentLimits.SupplyCurrentLimit = units::ampere_t{70};
  config.CurrentLimits.StatorCurrentLimit = units::ampere_t{120};
}

void NeoKrakenModule::setModuleState(frc::SwerveModuleState state) {
  double currentMeasurement = getEncoderPosition();
  frc::Rotation2d currentAngle{units::radian_t{currentMeasurement}};
  state.Optimize(currentAngle);

  units::meters_per_second_t speed = state.speed;
  units::radian_t angle = state.angle.Radians();
  setPoint = angle.value();

  double drivePercent = driveController.Calculate(getVelocity().value(), speed.value());
  double steerPercent = steerController.Calculate(currentMeasurement, angle.value());

  driveMotor.Set(drivePercent + ff.Calculate(speed).value());
  steerMotor.Set(-steerPercent);
}

double NeoKrakenModule::getEncoderPosition() {
  ctre::phoenix6::StatusSignal<units::turn_t> angle = encoderObject.GetAbsolutePosition();
  return (angle.GetValueAsDouble() * kCanCoderMultiplier) - offset;
}

double NeoKrakenModule::getPosition() {
  ctre::phoenix6::StatusSignal<units::turn_t> position = driveMotor.GetPosition();
  return position.GetValue().value() * kPositionMultiplier;
}

frc::SwerveModuleState NeoKrakenModule::getModuleState() {
  return {getVelocity(), frc::Rotation2d{units::radian_t{getEncoderPosition()}}};
}

frc::SwerveModulePosition NeoKrakenModule::getModulePosition() {
  return {units::meter_t{getPosition()}, units::radian_t{M_PI + getEncoderPosition()}};
}

units::meters_per_second_t NeoKrakenModule::getVelocity() {
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocity = driveMotor.GetVelocity();
  return units::meters_per_second_t{velocity.GetValue().value() * kVelocityMultiplier};
}