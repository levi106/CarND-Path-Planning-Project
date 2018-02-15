#pragma once
#include <map>
#include "input_params.hpp"
#include "vehicle_state.hpp"
#include "trajectory_point.hpp"

class Vehicle {
public:
  Vehicle();
  void ChooseNextState(InputParams& params);

  VehicleState state_;
  double ref_x_;
  double ref_y_;
  double ref_yaw_;
  double ref_v_;
  double ref_x_prev_;
  double ref_y_prev_;
  double s_;
  double car_s_;
  int lane_;
  double time_;
  double target_v_;
  int target_lane_;

private:
  void UpdateParameters(InputParams& params);
  void UpdateLane();
  void UpdateVelocity();
  std::vector<VehicleState> SuccessorStates() const;
  std::vector<double> GetKinematics(std::vector<SensorFusion> sensor_fusions, int lane) const;
  std::vector<TrajectoryPoint> GenerateTrajectory(VehicleState state, std::vector<SensorFusion> sensor_fusions) const;
  std::vector<TrajectoryPoint> KeepLaneTrajectory(VehicleState state, std::vector<SensorFusion> sensor_fusions) const;
  std::vector<TrajectoryPoint> PrepLaneChangeTrajectory(VehicleState state, std::vector<SensorFusion> sensor_fusions) const;
  std::vector<TrajectoryPoint> LaneChangeTrajectory(VehicleState state, std::vector<SensorFusion> sensor_fusions) const;
};