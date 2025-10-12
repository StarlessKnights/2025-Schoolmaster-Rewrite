// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <optional>

#include "grpl/LaserCan.h"

class TurboLaserCAN {
 private:
  grpl::LaserCan laserCan;

 public:
  explicit TurboLaserCAN(int id) : laserCan(id) {}

  int GetProximity() {
    std::optional<grpl::LaserCanMeasurement> measurement = laserCan.get_measurement();
    if (measurement && measurement->status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement->distance_mm;
    } else {
      return 1000;
    }
  }
};
