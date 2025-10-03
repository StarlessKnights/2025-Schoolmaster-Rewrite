// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include "constants/Constants.h"
#include "frc/DutyCycleEncoder.h"
#include "frc/controller/PIDController.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"

class AlgaeGrabberSubsystem : public frc2::SubsystemBase {
 private:
  rev::spark::SparkMax pivotMotor{AlgaeGrabberSubsystemsConstants::kPivotMotorID,
                                  rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax spinMotor{AlgaeGrabberSubsystemsConstants::kSpinMotorID,
                                 rev::spark::SparkLowLevel::MotorType::kBrushless};

  frc::DutyCycleEncoder thruBore{AlgaeGrabberSubsystemsConstants::kThruBoreEncoderID};

  frc::PIDController controller{1.0, 0, 0};

 public:
  AlgaeGrabberSubsystem();

  void ConfigurePivotMotor();
  void ConfigureSpinMotor();

  void SetSpinMotor(double percent);
  void SetPivotMotor(double percent);

  double LinearizeEncoderOutput(double currentPosition);

  void SetPosition(double position);

  double GetPosition();
  double GetLinearizedPosition();

  double GetSpinMotorCurrentDraw();

  bool IsAlgaeGrabberPIDAtSetpoint();

  void StopAll();
};
