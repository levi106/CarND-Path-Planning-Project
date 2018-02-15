#pragma once
#include <vector>
#include "sensor_fusion.hpp"

struct InputParams {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  std::vector<SensorFusion> sensor_fusions;
};
