#pragma once

#include "constants/Constants.h"
#include "frc/controller/PIDController.h"
#include "frc/simulation/DCMotorSim.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc2/command/SubsystemBase.h"
#include "rev/SparkClosedLoopController.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "rev/SparkRelativeEncoder.h"
#include "units/length.h"
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

  // Simulation
  static constexpr units::meter_t kDrumRadius = 0.02_m;
  static constexpr double kGearing = 15.0;
  static constexpr units::kilogram_t kCarriageMass = 15_kg;

  frc::PIDController m_simPID{30, 0.0, 0.0};

  frc::LinearSystem<2, 1, 2> m_elevatorPlant =
      frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(2), kCarriageMass, kDrumRadius, kGearing);

  frc::sim::DCMotorSim m_elevatorSim{m_elevatorPlant, frc::DCMotor::NEO(2)};

  units::length::meter_t simPosition = 0_m;

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

  units::length::meter_t EncoderRotationToMeters(double rotations) const {
    return (rotations / kGearing) * (2 * std::numbers::pi * kDrumRadius);
  }

  void SimulationPeriodic() override;
};
