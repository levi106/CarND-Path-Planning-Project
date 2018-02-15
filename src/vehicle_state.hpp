#pragma once

enum VehicleState {
  KeepLane,
  LaneChangeLeft,
  LaneChangeRight
};

inline int GetLaneDirection(VehicleState state) {
  switch (state) {
  case KeepLane:
    return 0;
  case LaneChangeLeft:
    return -1;
  case LaneChangeRight:
    return 1;
  }
}