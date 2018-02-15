#include "utils.hpp"

double distance(double x1, double y1, double x2, double y2) { return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)); }

void GlobalCoordinateToVehicleCoordinate(
  std::vector<double>& global_x,
  std::vector<double>& global_y,
  std::vector<double>& car_x,
  std::vector<double>& car_y,
  double ref_x,
  double ref_y,
  double ref_yaw
) {
  for (int i = 0; i < global_x.size(); i++) {
    double shift_x = global_x[i] - ref_x;
    double shift_y = global_y[i] - ref_y;
    car_x.push_back(shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
    car_y.push_back(shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
  }
}

void VechicleCoordinateToGlobalCoordinate(
  double vehicle_x,
  double vehicle_y,
  double& global_x,
  double& global_y,
  double ref_x,
  double ref_y,
  double ref_yaw
) {
  global_x = vehicle_x * cos(ref_yaw) - vehicle_y * sin(ref_yaw) + ref_x;
  global_y = vehicle_x * sin(ref_yaw) + vehicle_y * cos(ref_yaw) + ref_y;
}
