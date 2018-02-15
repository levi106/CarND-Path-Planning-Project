#pragma once
#include <vector>
#include <string>
#include "waypoint_map.hpp"
#include "sensor_fusion.hpp"
#include "vehicle.hpp"
#include "input_params.hpp"

class PathPlanner {
public:
  PathPlanner();
  void LoadWaypointMap(std::string& map_file);
  void Fit(InputParams &params);
  void Predict(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);

private:
  Vehicle vehicle_;
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  WaypointMap waypoint_map_;
  InputParams params_;

  void CalculatePts(std::vector<double>& car_ptsx, std::vector<double>& car_ptsy);
};
