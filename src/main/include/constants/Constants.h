// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <string>

#include "frc/geometry/Transform3d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"

namespace RobotConstants {
inline constexpr int kNominalVoltage = 12;
}

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kOperatorControllerPort = 1;

} // namespace OperatorConstants

namespace NeoKrakenModuleConstants {

inline constexpr double kNominalVoltage = 12.8;

} // namespace NeoKrakenModuleConstants

namespace DriveSubsystemConstants {
inline constexpr int kPigeonID = 10;
inline const std::string kCanivoreName = "CANIVORE";

inline constexpr double kFrontLeftDriveID = 1;
inline constexpr double kFrontLeftSteerID = 2;
inline constexpr double kFrontLeftEncoderID = 50;

inline constexpr double kFrontRightDriveID = 3;
inline constexpr double kFrontRightSteerID = 4;
inline constexpr double kFrontRightEncoderID = 51;

inline constexpr double kBackLeftDriveID = 5;
inline constexpr double kBackLeftSteerID = 6;
inline constexpr double kBackLeftEncoderID = 52;

inline constexpr double kBackRightDriveID = 7;
inline constexpr double kBackRightSteerID = 8;
inline constexpr double kBackRightEncoderID = 53;

inline constexpr double kFrontLeftOffset = 1.919009965644937 + M_PI;
inline constexpr double kFrontRightOffset = 2.049398332615217;
inline constexpr double kBackLeftOffset = -0.578310757032887 + M_PI;
inline constexpr double kBackRightOffset = -2.581689666011534 + M_PI;

inline constexpr double kRobotLength = .5525;
inline constexpr double kRobotWidth = .5525;

inline constexpr units::meters_per_second_t kMaxLinearSpeed = 4.5_mps;
inline constexpr units::radians_per_second_t kMaxAngularSpeed = 570_deg_per_s;

inline constexpr frc::Translation2d kModulePositions[] = {
    {units::meter_t{kRobotLength / 2}, units::meter_t{kRobotWidth / 2}},  // Front Left
    {units::meter_t{kRobotLength / 2}, units::meter_t{-kRobotWidth / 2}}, // Front Right
    {units::meter_t{-kRobotLength / 2}, units::meter_t{kRobotWidth / 2}}, // Back Left
    {units::meter_t{-kRobotLength / 2}, units::meter_t{-kRobotWidth / 2}} // Back Right
};

inline frc::SwerveDriveKinematics<4> kKinematics{kModulePositions[0], kModulePositions[1], kModulePositions[2],
                                                 kModulePositions[3]};
} // namespace DriveSubsystemConstants

namespace CameraConstants {
inline const std::string kPathToAprilTagLayout = "/home/lvuser/deploy/files/2025-reefscape-welded.json";

inline const std::string kLocalizationCamOneName = "lc1";
inline const std::string kLocalizationCamTwoName = "lc2";

inline const frc::Transform3d kLocalizationCamOneOffset{frc::Translation3d(-0.0952_m, 0.2921_m, 0.1_m),
                                                        frc::Rotation3d(0.0_rad, 0.0_rad, 0.0_rad)};

inline const frc::Transform3d kLocalizationCamTwoOffset{
    frc::Translation3d(0.2159_m, -0.279_m, 0.1143_m),
    frc::Rotation3d(0_rad, units::radian_t{units::degree_t{20}}, units::radian_t{units::degree_t{37}})};
} // namespace CameraConstants

namespace ElevatorSubsystemConstants {
inline constexpr int kPrimaryMotorID = 51;
inline constexpr int kSecondaryMotorID = 50;
inline constexpr int kCoralMotorID = 58;

inline constexpr int kCoralSensorID = 37;
inline constexpr int kCoralSensorProximityThreshold = 20;

inline constexpr int kNEO550CurrentLimit = 30;

inline constexpr double kL1EncoderPosition = 0.0;
inline constexpr double kL2EncoderPosition = 8.5;
inline constexpr double kL3EncoderPosition = 25.0;
inline constexpr double kL4EncoderPosition = 52.0;

inline constexpr double kHPEncoderPosition = 0.1;
inline constexpr double kDefaultEncoderPosition = 1.5; // Prevents carriage from hitting base

inline constexpr double kHighAlgaePosition = 44.0;
inline constexpr double kLowAlgaePosition = 28.0;
inline constexpr double kProcessorScorePosition = 4.0;
inline constexpr double kGroundIntakePosition = 1.5;

inline constexpr double kGrabberSpeed = 0.2;
inline constexpr double kL1GrabberSpeed = 0.15;
inline constexpr double kL4GrabberSpeed = 0.30;
inline constexpr double kAutoL4GrabberSpeed = 0.25;

inline constexpr double kIntakeGrabberSpeed = 0.2;

inline constexpr double kHomedCurrentDraw = 60.0;

inline constexpr double kMaxAcceleration = 6000.0;
inline constexpr double kMaxVelocity = 15000.0;
inline constexpr double kAtSetpointTolerance = 1.0;

inline constexpr double kArbitraryFeedforward = 0.44;
} // namespace ElevatorSubsystemConstants
