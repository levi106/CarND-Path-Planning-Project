#include "vehicle.hpp"
#include <iostream>
#include <algorithm>
#include <math.h>
#include "cost.hpp"
#include "constants.hpp"
#include "utils.hpp"

Vehicle::Vehicle()
  : state_(KeepLane)
  , ref_x_(0)
  , ref_y_(0)
  , ref_yaw_(0)
  , ref_v_(0)
  , ref_x_prev_(0)
  , ref_y_prev_(0)
  , lane_(kTargetLane)
  , s_(0)
  , car_s_(0)
  , time_(0)
  , target_v_(kTargetVelocity)
  , target_lane_(kTargetLane) {
}

void Vehicle::UpdateParameters(InputParams& params) {
  int previous_path_size = params.previous_path_x.size();

  if (previous_path_size < 2) {
    ref_x_ = params.car_x;
    ref_y_ = params.car_y;
    ref_yaw_ = DegreeToRadian(params.car_yaw);
    ref_x_prev_ = params.car_x - cos(ref_yaw_);
    ref_y_prev_ = params.car_y - sin(ref_yaw_);
  } else {
    ref_x_ = params.previous_path_x[previous_path_size-1];
    ref_y_ = params.previous_path_y[previous_path_size-1];
    ref_x_prev_ = params.previous_path_x[previous_path_size-2];
    ref_y_prev_ = params.previous_path_y[previous_path_size-2];
    ref_yaw_ = atan2(ref_y_ - ref_y_prev_, ref_x_ - ref_x_prev_);
  }

  s_ = previous_path_size > 0 ? params.end_path_s : params.car_s;
  car_s_ = params.car_s;
  time_ = (double)params.previous_path_x.size() * kSecondsPerFrame;
}

void Vehicle::ChooseNextState(InputParams& params) {
  UpdateParameters(params);

  double cost;
  std::vector<double> costs;
  std::vector<std::vector<TrajectoryPoint>> final_trajectories;
  std::vector<VehicleState> states = SuccessorStates();
  for (auto state : states) {
    std::vector<TrajectoryPoint> trajectory = GenerateTrajectory(state, params.sensor_fusions);
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, params.sensor_fusions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }
  auto best_cost = std::min_element(std::begin(costs), std::end(costs));
  int best_idx = std::distance(std::begin(costs), best_cost);
  TrajectoryPoint& best_trajectory = final_trajectories[best_idx][1];

  state_ = best_trajectory.state_;
  ref_v_ = best_trajectory.v_;
  lane_ = best_trajectory.lane_;
}

std::vector<VehicleState> Vehicle::SuccessorStates() const {
  std::vector<VehicleState> states;
  states.push_back(KeepLane);
  if (state_ == KeepLane) {
    if (lane_ != kLaneNum - 1) {
      states.push_back(LaneChangeRight);
    }
    if (lane_ != 0) {
      states.push_back(LaneChangeLeft);
    }
  }
  return states;
}

std::vector<double> Vehicle::GetKinematics(std::vector<SensorFusion> sensor_fusions, int lane) const {
  SensorFusion vehicle_ahead;
  SensorFusion vehicle_behind;

  double new_v;
  if (SensorFusion::GetVehicleAhead(sensor_fusions, lane, s_, time_, 30, vehicle_ahead)
    && vehicle_ahead.v < ref_v_) {
    std::cout << "[lane " << lane << "] found a vehicle ahead (" << (vehicle_ahead.s + vehicle_ahead.v * time_) - s_ << ")" << std::endl;
    new_v = ref_v_ - 0.224;
  } else if (SensorFusion::GetVehicleAhead(sensor_fusions, lane, car_s_, 0, 30, vehicle_ahead)) {
    std::cout << "[lane " << lane << "] found a vehicle ahead (" << (vehicle_ahead.s) - car_s_ << ")" << std::endl;
    new_v = ref_v_ - 0.224;
  } else {
    if (ref_v_ < target_v_) {
      new_v = ref_v_ + 0.224;
    } else {
      new_v = ref_v_;
    }
  }
  return {new_v};
}

std::vector<TrajectoryPoint> Vehicle::GenerateTrajectory(VehicleState state, std::vector<SensorFusion> sensor_fusions) const {
  switch (state) {
  case KeepLane:
    return KeepLaneTrajectory(state, sensor_fusions);
  case LaneChangeLeft:
  case LaneChangeRight:
    return LaneChangeTrajectory(state, sensor_fusions);
  }
}

std::vector<TrajectoryPoint> Vehicle::KeepLaneTrajectory(VehicleState state, std::vector<SensorFusion> sensor_fusions) const {
  std::vector<TrajectoryPoint> trajectory;

  trajectory.push_back(TrajectoryPoint(lane_, ref_v_, state_));
  std::vector<double> kinematics = GetKinematics(sensor_fusions, lane_);
  double new_v = kinematics[0];
  trajectory.push_back(TrajectoryPoint(lane_, new_v, state));
  return trajectory;
}

std::vector<TrajectoryPoint> Vehicle::LaneChangeTrajectory(VehicleState state, std::vector<SensorFusion> sensor_fusions) const {
  std::vector<TrajectoryPoint> trajectory;
  SensorFusion fusion;
  int new_lane = lane_ + GetLaneDirection(state);

  if (new_lane < 0 || new_lane >= kLaneNum) {
    return trajectory;
  }
  if (SensorFusion::GetVehicleAhead(sensor_fusions, new_lane, s_, time_, 30, fusion)) {
    std::cout << "[lane " << new_lane << "] found a vehicle ahead (" << (fusion.s + fusion.v * time_) - s_ << ")" << std::endl;
    return trajectory;
  }
  if (SensorFusion::GetVehicleBehind(sensor_fusions, new_lane, s_, time_, 20, fusion)) {
    std::cout << "[lane " << new_lane << "] found a vehicle behind (" << s_ - (fusion.s + fusion.v * time_) << ")" << std::endl;
    return trajectory;
  }
  trajectory.push_back(TrajectoryPoint(lane_, ref_v_, state_));
  std::vector<double> kinematics = GetKinematics(sensor_fusions, new_lane);
  trajectory.push_back(TrajectoryPoint(new_lane, kinematics[0], state));
  return trajectory;
}
