#pragma once

namespace CommandFactory {
inline frc2::CommandPtr ElevatorPopUpAndAlgaeGrabberGoToPosition(AlgaeGrabberSubsystem* grabber,
                                                                 ElevatorSubsystem* elevator,
                                                                 const double algaeGrabberEncoderPosition) {
  double homePosition = 0.0;

  return frc2::FunctionalCommand(
             [=, &homePosition]() mutable {
               const double currentPosition = elevator->GetPosition();
               constexpr double minPosition = AlgaeGrabberSubsystemsConstants::kMinimumSafeElevatorEncoderPosition;

               grabber->SetPosition(algaeGrabberEncoderPosition);
               homePosition = std::max(currentPosition, minPosition);
             },
             [=, &homePosition] {
               elevator->SetPosition(homePosition);

               if (elevator->IsElevatorPIDAtSetpoint()) {
                 grabber->SetPosition(algaeGrabberEncoderPosition);
               } else {
                 grabber->SetPivotMotor(0.0);
               }
             },
             [=](bool) {
               elevator->StopAll();
               grabber->StopAll();
             },
             [=] { return grabber->IsAlgaeGrabberPIDAtSetpoint(); }, {grabber, elevator})
      .ToPtr()
      .WithName("ElevatorPopUpAndAlgaeGrabberGoToPosition");
}
}  // namespace CommandFactory