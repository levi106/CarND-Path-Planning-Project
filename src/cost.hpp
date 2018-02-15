#pragma once
#include <vector>
#include "sensor_fusion.hpp"
#include "trajectory_point.hpp"
#include "vehicle.hpp"

double calculate_cost(const Vehicle& vehicle, const std::vector<SensorFusion>& sensor_fusions, const std::vector<TrajectoryPoint>& trajectory);
