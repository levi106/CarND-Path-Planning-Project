#pragma once
#include "vehicle_state.hpp"

struct TrajectoryPoint {
  int lane_;
  double v_;
  VehicleState state_;

  TrajectoryPoint(int lane, double v, VehicleState state)
    : lane_(lane), v_(v), state_(state)
  {}
};