#include "path_planner.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>
#include "spline.h"
#include "utils.hpp"
#include "constants.hpp"

PathPlanner::PathPlanner() {
}

void PathPlanner::LoadWaypointMap(std::string& map_file) {
  waypoint_map_.load(map_file);
}

void PathPlanner::CalculatePts(std::vector<double>& car_ptsx, std::vector<double>& car_ptsy) {
  std::vector<double> global_ptsx;
  std::vector<double> global_ptsy;

  global_ptsx.push_back(vehicle_.ref_x_prev_);
  global_ptsx.push_back(vehicle_.ref_x_);
  global_ptsy.push_back(vehicle_.ref_y_prev_);
  global_ptsy.push_back(vehicle_.ref_y_);

  std::vector<double> next_wp0 = waypoint_map_.getXY(vehicle_.s_+30, (2+4*vehicle_.lane_));
  std::vector<double> next_wp1 = waypoint_map_.getXY(vehicle_.s_+60, (2+4*vehicle_.lane_));
  std::vector<double> next_wp2 = waypoint_map_.getXY(vehicle_.s_+90, (2+4*vehicle_.lane_));

  global_ptsx.push_back(next_wp0[0]);
  global_ptsx.push_back(next_wp1[0]);
  global_ptsx.push_back(next_wp2[0]);

  global_ptsy.push_back(next_wp0[1]);
  global_ptsy.push_back(next_wp1[1]);
  global_ptsy.push_back(next_wp2[1]);

  GlobalCoordinateToVehicleCoordinate(global_ptsx, global_ptsy, car_ptsx, car_ptsy, vehicle_.ref_x_, vehicle_.ref_y_, vehicle_.ref_yaw_);
}

void PathPlanner::Fit(InputParams& params) {
  params_ = params;
}

void PathPlanner::Predict(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals) {
  vehicle_.ChooseNextState(params_);

  std::vector<double> ptsx, ptsy;
  CalculatePts(ptsx, ptsy);

  tk::spline s;
  s.set_points(ptsx, ptsy);

  // copy from previous path
  std::copy(params_.previous_path_x.begin(), params_.previous_path_x.end(), std::back_inserter(next_x_vals));
  std::copy(params_.previous_path_y.begin(), params_.previous_path_y.end(), std::back_inserter(next_y_vals));

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = distance(target_x, target_y, 0, 0);
  double x_add_on = 0;
  for (int i = 1; i <= kWaypoints - params_.previous_path_x.size(); i++) {
    double N = (target_dist / Mph2Mps(kSecondsPerFrame * vehicle_.ref_v_));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);
    x_add_on = x_point;

    double global_x, global_y;
    VechicleCoordinateToGlobalCoordinate(x_point, y_point, global_x, global_y, vehicle_.ref_x_, vehicle_.ref_y_, vehicle_.ref_yaw_);

    next_x_vals.push_back(global_x);
    next_y_vals.push_back(global_y);
  }
}