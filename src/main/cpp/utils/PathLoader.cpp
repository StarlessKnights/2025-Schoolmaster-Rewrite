// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include "utils/PathLoader.hpp"

#include <functional>
#include <memory>

#include "frc/DriverStation.h"
#include "frc/RobotBase.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/config/PIDConstants.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"

void PathLoader::ConfigurePathPlanner(DriveSubsystem& drive, TurboPoseEstimator& poseEstimator) {
  std::function<void(const frc::Pose2d&)> resetPose = [&](const frc::Pose2d& pose) {
    poseEstimator.ResetEstimatorPosition(drive.GetAngle(), drive.GetModulePositions(), pose);
  };
  std::function<void(frc::ChassisSpeeds)> driveFunction = [&](frc::ChassisSpeeds speeds) { drive.AutoDrive(speeds); };
  std::function<frc::Pose2d()> poseSupplier = [&]() { return drive.GetPose(); };
  std::function<bool()> shouldFlipPaths = []() {
    return frc::DriverStation::GetAlliance() == frc::DriverStation::kRed;
  };
  std::function<frc::ChassisSpeeds()> robotRelativeSpeeds = [&]() { return drive.GetRobotRelativeChassisSpeeds(); };

  std::shared_ptr<pathplanner::PPHolonomicDriveController> driveController;

  if (frc::RobotBase::IsReal()) {
    driveController = std::make_shared<pathplanner::PPHolonomicDriveController>(
        pathplanner::PIDConstants(3.0, 0.0, 0.0), pathplanner::PIDConstants(5.0, 0.0, 0.0));
  } else {
    driveController = std::make_shared<pathplanner::PPHolonomicDriveController>(
        pathplanner::PIDConstants(0.15, 0.0, 0.0), pathplanner::PIDConstants(0.15, 0.0, 0.0));
  }

  pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

  pathplanner::AutoBuilder::configure(poseSupplier, resetPose, robotRelativeSpeeds, driveFunction, driveController,
                                      config, shouldFlipPaths, &drive);
}
