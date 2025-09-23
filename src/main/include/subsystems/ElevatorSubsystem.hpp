#pragma once

#include "constants/Constants.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "utils/TurboLaserCAN.hpp"

class ElevatorSubsystem : public frc2::SubsystemBase {
private:
  rev::spark::SparkMax primaryMotor{ElevatorSubsystemConstants::kPrimaryMotorID,
                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax secondaryMotor{ElevatorSubsystemConstants::kSecondaryMotorID,
                                      rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax spinGrabberMotor{ElevatorSubsystemConstants::kSpinGrabberMotorID,
                                        rev::spark::SparkLowLevel::MotorType::kBrushless};

  TurboLaserCAN laserCan{ElevatorSubsystemConstants::kCoralSensorID};

  rev::RelativeEncoder &primaryEncoder = primaryMotor.GetEncoder();
  rev::spark::SparkClosedLoopController &onboardClosedLoop = primaryMotor.GetClosedLoopController();

  double currentSetpoint = 0.0;

public:
  ElevatorSubsystem();
  void ConfigureMotors();
  void SetPosition(double position);
  double GetPosition();
  bool IsElevatorPIDAtSetpoint();
};