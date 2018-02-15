#include "sensor_fusion.hpp"
#include <limits>

bool SensorFusion::IsAhead(int lane, int car_s, double time, double distance) const {
  if (IsInLane(lane)) {
    double target_s = s + time * v;
    if ((target_s > car_s) && ((target_s - car_s) < distance)) {
      return true;
    }
  }
  return false;
}

bool SensorFusion::IsBehind(int lane, int car_s, double time, double distance) const {
  if (IsInLane(lane)) {
    double target_s = s + time * v;
    if ((target_s < car_s) && ((car_s - target_s) < distance)) {
      return true;
    }
  }
  return false;
}

bool SensorFusion::GetVehicleAhead(const std::vector<SensorFusion>& sensor_fusions, int lane, int car_s, double time, double distance, SensorFusion& sensor_fusion) {
  int min_s = std::numeric_limits<int>::max();
  bool found = false;
  for (const auto fusion : sensor_fusions) {
    if (fusion.IsAhead(lane, car_s, time, distance)) {
      double s = fusion.s + time * fusion.v;
      if (s < min_s) {
        sensor_fusion = fusion;
        min_s = s;
        found = true;
      }
    }
  }
  return found;
}

bool SensorFusion::GetVehicleBehind(const std::vector<SensorFusion>& sensor_fusions, int lane, int car_s, double time, double distance, SensorFusion& sensor_fusion) {
  int max_s = std::numeric_limits<int>::min();
  bool found = false;
  for (const auto fusion : sensor_fusions) {
    if (fusion.IsBehind(lane, car_s, time, distance)) {
      double s = fusion.s + time * fusion.v;
      if (s > max_s) {
        sensor_fusion = fusion;
        max_s = s;
        found = true;
      }
    }
  }
  return found;
}
