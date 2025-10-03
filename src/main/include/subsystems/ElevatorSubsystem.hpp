// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "constants/Constants.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "rev/SparkRelativeEncoder.h"
#include "utils/TurboLaserCAN.hpp"

class ElevatorSubsystem : public frc2::SubsystemBase {
 private:
  rev::spark::SparkMax primaryMotor{ElevatorSubsystemConstants::kPrimaryMotorID,
                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax secondaryMotor{ElevatorSubsystemConstants::kSecondaryMotorID,
                                      rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax coralMotor{ElevatorSubsystemConstants::kCoralMotorID,
                                  rev::spark::SparkLowLevel::MotorType::kBrushless};

  TurboLaserCAN coralSensor{ElevatorSubsystemConstants::kCoralSensorID};

  rev::spark::SparkRelativeEncoder primaryEncoder = primaryMotor.GetEncoder();
  rev::spark::SparkClosedLoopController onboardClosedLoop = primaryMotor.GetClosedLoopController();

  double currentSetpoint = 0.0;

 public:
  ElevatorSubsystem();

  void ConfigurePrimaryMotor();
  void ConfigureSecondaryMotor();
  void ConfigureCoralMotor();

  void SetSpin(double percent);
  void SetCoralGrabber(double percent);

  double GetPosition();
  void SetPosition(double position);

  void ZeroEncoder();
  void ZeroEncoder(double customZero);

  void StopAll();

  bool GetIsCoralInHoldingPosition();
  bool IsElevatorPIDAtSetpoint();

  double GetElevatorCurrentDraw();
};
