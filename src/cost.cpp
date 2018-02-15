#include "cost.hpp"
#include <math.h>

double calculate_cost(const Vehicle& vehicle, const std::vector<SensorFusion>& sensor_fusions, const std::vector<TrajectoryPoint>& trajectory) {
  double distance_cost = abs(vehicle.target_lane_ - trajectory[1].lane_);
  double inefficiency_cost = abs(vehicle.target_v_ - trajectory[1].v_);
  double cost = distance_cost + 10 * inefficiency_cost;
  return cost;
}