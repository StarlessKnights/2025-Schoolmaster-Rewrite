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

#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
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

inline frc::SwerveDriveKinematics<4> swerveKinematics{kModulePositions[0], kModulePositions[1], kModulePositions[2],
                                                      kModulePositions[3]};
} // namespace DriveSubsystemConstants
