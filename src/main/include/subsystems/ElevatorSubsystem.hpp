#pragma once

#include "constants/Constants.h"
#include "frc/smartdashboard/Mechanism2d.h"
#include "frc/smartdashboard/MechanismLigament2d.h"
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
  double simulatedPosition = 0.0;

  frc::Mechanism2d elevatorMech{0.5, 2.0};
  frc::MechanismRoot2d *elevatorRoot = elevatorMech.GetRoot("Elevator Root", 0.25, 0.1);
  frc::MechanismLigament2d *m_elevator = elevatorRoot->Append<frc::MechanismLigament2d>("elevator", 1, 90_deg);

public:
  ElevatorSubsystem();
  void ConfigureMotors();
  void SetPosition(double position);
  double GetPosition();
  bool IsElevatorPIDAtSetpoint();

  void Periodic() override;
  void SimulationPeriodic() override;
};