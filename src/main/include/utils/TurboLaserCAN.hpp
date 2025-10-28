// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <optional>

#include "grpl/LaserCan.h"

class TurboLaserCAN {
  grpl::LaserCan laserCan;

 public:
  explicit TurboLaserCAN(const int id) : laserCan(id) {}

  [[nodiscard]]
  int GetProximity() const {
    if (const std::optional<grpl::LaserCanMeasurement> measurement = laserCan.get_measurement();
        measurement && measurement->status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement->distance_mm;
    }

    return 1000;
  }
};
