#pragma once

#include "grpl/LaserCan.h"
#include <optional>

class TurboLaserCAN {
private:
  grpl::LaserCan laserCan;

public:
  TurboLaserCAN(int id) : laserCan(id) {}

  int getProximity() {
    std::optional<grpl::LaserCanMeasurement> measurement = laserCan.get_measurement();
    if (measurement && measurement->status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement->distance_mm;
    } else {
      return 1000;
    }
  }
};