#include "waypoint_map.hpp"
#include <fstream>
#include <sstream>
#include "utils.hpp"

void WaypointMap::load(std::string& map_file) {
  std::ifstream in_map(map_file.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x_.push_back(x);
    map_waypoints_y_.push_back(y);
    map_waypoints_s_.push_back(s);
    map_waypoints_dx_.push_back(d_x);
    map_waypoints_dy_.push_back(d_y);
  }
}

std::vector<double> WaypointMap::getXY(double s, double d) {
  int prev_wp = -1;

  while (s > map_waypoints_s_[prev_wp + 1] && (prev_wp < (int)(map_waypoints_s_.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % map_waypoints_x_.size();
  double heading = atan2((map_waypoints_y_[wp2] - map_waypoints_y_[prev_wp]),
                         (map_waypoints_x_[wp2] - map_waypoints_x_[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - map_waypoints_s_[prev_wp]);

  double seg_x = map_waypoints_x_[prev_wp] + seg_s * cos(heading);
  double seg_y = map_waypoints_y_[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x,y};
}
