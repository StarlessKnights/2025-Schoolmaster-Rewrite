#pragma once

#include "constants/Constants.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
private:
  rev::spark::SparkMax primaryMotor{ElevatorSubsystemConstants::kPrimaryMotorID,
                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax secondaryMotor{ElevatorSubsystemConstants::kSecondaryMotorID,
                                      rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax spinGrabberMotor{ElevatorSubsystemConstants::kSpinGrabberMotorID,
                                        rev::spark::SparkLowLevel::MotorType::kBrushless};
};