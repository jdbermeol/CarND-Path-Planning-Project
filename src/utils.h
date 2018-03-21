#ifndef UTILS_H
#define UTILS_H
#include <math.h>
#include <vector>
#include <iostream>
#include "vehicle.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

Vehicle build_vehicle_from_sensor_fusion(const std::vector<double>& sensor_fusion);

Vehicle predict_sensor_future_position(const Vehicle& sensor_fusion, int N, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<Vehicle> get_vehicles_ahead(const std::vector<Vehicle>& vehicles, double lane, double width, double s);

int compute_car_lane(double d);

#endif