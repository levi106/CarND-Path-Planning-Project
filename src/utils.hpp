#pragma once
#include <math.h>
#include <vector>

constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi()/ 180; }
inline double DegreeToRadian(double x) { return x * pi()/ 180; }
inline double rad2ged(double x) { return x * 180 / pi(); }
inline double Mph2Mps(double mph) { return mph * 0.45; }
inline double Mps2Mph(double mps) { return mps * 2.24; }

double distance(double x1, double y1, double x2, double y2);

// convert from global coordinate system to vehicle coordinate system
void GlobalCoordinateToVehicleCoordinate(
  std::vector<double>& global_x,
  std::vector<double>& global_y,
  std::vector<double>& vehicle_x,
  std::vector<double>& vehicle_y,
  double ref_x,
  double ref_y,
  double ref_yaw
);

// convert from vechicle coordinate system to global coordinate system
void VechicleCoordinateToGlobalCoordinate(
  double vehicle_x,
  double vehicle_y,
  double& global_x,
  double& global_y,
  double ref_x,
  double ref_y,
  double ref_yaw
);