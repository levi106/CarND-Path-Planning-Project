#pragma once
#include <string>
#include <vector>

class WaypointMap {
public:
  void load(std::string& map_file);
  std::vector<double> getXY(double s, double d);

private:
  std::vector<double> map_waypoints_x_;
  std::vector<double> map_waypoints_y_;
  std::vector<double> map_waypoints_s_;
  std::vector<double> map_waypoints_dx_;
  std::vector<double> map_waypoints_dy_;
};
