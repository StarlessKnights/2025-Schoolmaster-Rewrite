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
namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

} // namespace OperatorConstants

namespace NeoKrakenModuleConstants {

inline constexpr double kNominalVoltage = 12.8;

} // namespace NeoKrakenModuleConstants

namespace DriveSubsystemConstants {
inline constexpr int kPigeonID = 10;
inline constexpr std::string kCanivoreName = "CANIVORE";

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
} // namespace DriveSubsystemConstants