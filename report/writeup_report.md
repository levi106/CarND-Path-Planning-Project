# **Path Planning Project**

**The Goal of this Project**

In this project, your goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

## Rubic Points

### Here I will consider the [rubic points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.

---

### Compilation

#### 1. The code compiles correctly

My code could be compiled without errors.

To enable debugging, I appended the following line to CMakeLists.txt:

```cmake
set(CMAKE_BUILD_TYPE Debug)
```

### Valid Trajectories

#### 2. The car is able to drive at least 4.32 miles

I confirmed that the car is able to drive 9.00 miles without exceeding acceleration, jerk, speed, collision and driving outside of the lanes.

#### 3. The car drives according to the speed limit.

My car obeys the 50 mph speed limit. I wrote the code to keep 49.5 mph much as possible.

#### 4. Max Acceleration and jerk are not Exceeded.

The car does not excceed a total acceleration of 10 m/s^2 and a jerk of 10 m/^s3.

#### 5. Car does not have collisions.

To avoid collisions, my car changes lanes or slows down according to a car ahead. The details will be described later.

#### 6. The car stays in its lane, except for the time between changing lanes.

The car doesn't spend more than a 3 second length out side the lane lines during changing lanes.

#### 7. The car is able to change lanes

The car is able to smoothly change lanes when behind a alower moving car.

### Reflection

#### 8. There is a reflection on how to generate paths.

I generated the path in the following steps:

1. A behavior planner returns the best state to induce the desired vehicle behavior.
2. Calculate new waypoints based on the velocity and the lane line derived from the next state.

**Behavior Planner**

The car has one of the following three states.

- KeepLane
- LaneChangeLeft
- LaneChangeRight

For each state, I calculate the lane line and the velocity that the car should be, compare these costs and make it transition to the state having the smallest cost. However, in the case of LaneChangeLeft and LaneChangeRight, when there is a vehicle 30m ahead or 20m behind the final lane, it does not transition to the state.

```cpp
  std::vector<TrajectoryPoint> trajectory;
  ...
  if (SensorFusion::GetVehicleAhead(sensor_fusions, new_lane, s_, time_, 30, fusion)) {
    // Returning an empty vector means that it will not transition to the state.
    return trajectory;
  }
  if (SensorFusion::GetVehicleBehind(sensor_fusions, new_lane, s_, time_, 20, fusion)) {
    // Returning an empty vector means that it will not transition to the state.
    return trajectory;
  }
```

Here, `time_` is the time it takes for the car to pass the previous path. In order to find the path point ahead of the last point predicted last cycle, I check whether there exists cars after `time_` time.

```cpp
  time_ = (double)params.previous_path_x.size() * kSecondsPerFrame;
```

It is checked whether or not a car is in a specific lane by using the sensor fusion value as follows:

```cpp
  inline bool IsInLane(int lane) const {
    if (d < (kLaneWidth * lane + kLaneWidth + kLaneWidth / 4) && d > kLaneWidth * lane - kLaneWidth / 4) {
      return true;
    } else {
      return false;
    }
  }
```

`d` is the car's d position in frenet coordinates. The quarter of the lane width is added or subtracted due to consider the case where the car is just changing the lane from the next.

The velocity for each state is calculated as follow:

```cpp
  double new_v;
  if (SensorFusion::GetVehicleAhead(sensor_fusions, lane, s_, time_, 30, vehicle_ahead)
    && vehicle_ahead.v < ref_v_) {
    new_v = ref_v_ - 0.224;
  } else if (SensorFusion::GetVehicleAhead(sensor_fusions, lane, car_s_, 0, 30, vehicle_ahead)) {
    new_v = ref_v_ - 0.224;
  } else {
    if (ref_v_ < target_v_) {
      new_v = ref_v_ + 0.224;
    } else {
      new_v = ref_v_;
    }
  }
```

If the car is in front, it decelerates and if it does not reach the target speed it is accelerating. In both cases, I make sure that the acceleration and jerk values do not exceed the limit.

Finally, I simply calculated the cost as follows. Since the value of Target Lane is set to 1, it is designed to run in the middle lane as much as possible.

$$
|TL - FL| + 10 \times |TV - V |
$$
- *TL* ... Target Lane
- *FL* ... Final Lane
- *TV* ... Target Velocity
- *V* ... Current Velocity

**Waypoints**

I used [spline tool](kluge.in-chemnitz.de/opensource/spline/) to get the spline curve fitting the previous points and the new points derived from the next state. Then I append new waypoints, until the new path has 50 total waypoints.

```cpp
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
```

