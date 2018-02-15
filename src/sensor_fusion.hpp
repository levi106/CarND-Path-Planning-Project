#pragma once
#include <vector>
#include "constants.hpp"

class SensorFusion {
public:
  double id;
  double map_x;
  double map_y;
  double vx;
  double vy;
  double v;
  double s;
  double d;

  inline bool IsInLane(int lane) const {
    if (d < (kLaneWidth * lane + kLaneWidth + kLaneWidth / 4) && d > kLaneWidth * lane - kLaneWidth / 4) {
      return true;
    } else {
      return false;
    }
  }

  bool IsAhead(int lane, int car_s, double time, double distance) const;
  bool IsBehind(int lane, int car_s, double time, double distance) const;

  static bool GetVehicleAhead(const std::vector<SensorFusion>& sensor_fusions, int lane, int car_s, double time, double distance, SensorFusion& sensor_fusion);
  static bool GetVehicleBehind(const std::vector<SensorFusion>& sensor_fusions, int lane, int car_s, double time, double distance, SensorFusion& sensor_fusion);
};
