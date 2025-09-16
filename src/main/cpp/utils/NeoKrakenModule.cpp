#include "constants/Constants.h"
#include "frc/controller/PIDController.h"
#include "frc/controller/SimpleMotorFeedforward.h"
#include "rev/SparkBase.h"
#include "rev/SparkMax.h"
#include "rev/config/SparkBaseConfig.h"
#include "units/base.h"
#include "units/length.h"
#include <units/velocity.h>
#include <units/voltage.h>
#include <utils/NeoKrakenModule.hpp>

void NeoKrakenModule::configPIDInternal() {
  this->ff = frc::SimpleMotorFeedforward<units::meters>{
      0.015_V,
      0.212_V / 1_mps};

  this->driveController = frc::PIDController(0.01, 0.0, 0.0);
  this->steerController = frc::PIDController(0.3, 0.0, 0.0);

  this->steerController.EnableContinuousInput(-M_PI, M_PI);
}

void NeoKrakenModule::configSteerMotor(rev::spark::SparkMax &target) {
  rev::spark::SparkBaseConfig config;
  config.Inverted(false);
  config.VoltageCompensation(NeoKrakenModuleConstants::kNominalVoltage);
  config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
  config.SmartCurrentLimit(80);
  target.Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);
}