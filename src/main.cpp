#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "utils.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

	// Reference velocity.
  double ref_vel = 0.0; // mph
	
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {          
        	// Main car's localization Data
					Vehicle vehicle;
					vehicle.id = 1;
					vehicle.x = j[1]["x"];
					vehicle.y = j[1]["y"];
					vehicle.s = j[1]["s"];
					vehicle.d = j[1]["d"];
					vehicle.v = j[1]["speed"];
					vehicle.yaw = deg2rad(j[1]["yaw"]);
					vehicle.lane = compute_car_lane(vehicle.d);


					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Provided previous path point size.
					int prev_size = previous_path_x.size();
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];


					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					vector<Vehicle> sensor_fusion;
					sensor_fusion.resize(j[1]["sensor_fusion"].size()); 
					transform (
						j[1]["sensor_fusion"].begin(), j[1]["sensor_fusion"].end(),
						sensor_fusion.begin(), build_vehicle_from_sensor_fusion);
					// Ignore cars going in the opposite direction
					sensor_fusion.erase(
						remove_if(
							sensor_fusion.begin(), 
              sensor_fusion.end(),
              [](const Vehicle& vehicle){return vehicle.d >= 11;}),
						sensor_fusion.end());


					// Compute reference car location
					vector<double> ptsx;
          vector<double> ptsy;
					Vehicle ref_vehicle;
					// Do I have have previous points
					if ( prev_size < 2 ) {
						// Use current car location
						ref_vehicle.x = vehicle.x;
						ref_vehicle.y = vehicle.y;
						ref_vehicle.yaw = vehicle.yaw;
						ref_vehicle.s = vehicle.s;
						ref_vehicle.d = vehicle.d;
						ref_vehicle.v = vehicle.v;
						ref_vehicle.lane = vehicle.lane;
						ptsx.push_back(ref_vehicle.x - cos(ref_vehicle.yaw));
						ptsx.push_back(ref_vehicle.x);
						ptsy.push_back(ref_vehicle.y - sin(ref_vehicle.yaw));
						ptsy.push_back(ref_vehicle.y);
					} else {
						// Use the last two points.
						ref_vehicle.x = previous_path_x[prev_size - 1];
						ref_vehicle.y = previous_path_y[prev_size - 1];
						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_vehicle.yaw = atan2(ref_vehicle.y-ref_y_prev, ref_vehicle.x-ref_x_prev);
						ref_vehicle.s = end_path_s;
						ref_vehicle.d = end_path_d;
						ref_vehicle.v = ref_vel;
						ref_vehicle.lane = compute_car_lane(ref_vehicle.d);
						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_vehicle.x);
						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_vehicle.y);
					}


					// Prediction: Calulate car position in the future
					vector<Vehicle> sensor_fusion_prediction;
					sensor_fusion_prediction.resize(sensor_fusion.size());
					transform (
						sensor_fusion.begin(), sensor_fusion.end(),
						sensor_fusion_prediction.begin(),
						[&prev_size,&map_waypoints_s,&map_waypoints_x,&map_waypoints_y](const Vehicle& vehicle){ return predict_sensor_future_position(vehicle, prev_size, map_waypoints_s, map_waypoints_x, map_waypoints_y); });

					// Prediction : Analysing other cars positions.
					bool car_ahead = false;
					bool car_left = false;
					bool car_right = false;
					double car_ahead_speed;
					for(auto const& sensor_fusion_car: sensor_fusion_prediction) {
						if (sensor_fusion_car.lane == ref_vehicle.lane) {
							if((sensor_fusion_car.s - ref_vehicle.s) > 0 && (sensor_fusion_car.s - ref_vehicle.s) < 30){// Car in our lane.
								car_ahead = true;
								car_ahead_speed =  sensor_fusion_car.v;
							}
						} else if ( sensor_fusion_car.lane - ref_vehicle.lane == 1 ) {
							car_right |= ref_vehicle.s - 30 < sensor_fusion_car.s && ref_vehicle.s + 30 > sensor_fusion_car.s;
						} else if ( ref_vehicle.lane - sensor_fusion_car.lane  == 1 ) {
							car_left |= ref_vehicle.s - 30 < sensor_fusion_car.s && ref_vehicle.s + 30 > sensor_fusion_car.s;
						}
          }
					

					// Behavior
          const double MAX_SPEED = 49.5;
          const double MAX_ACC = .224;
					double MIN_SPEED = 0;
					double speed_diff = MAX_ACC;
					if ( car_ahead ) { // Car ahead
						if ( !car_left && ref_vehicle.lane > 0 ) {
							ref_vehicle.lane--;
						} else if ( !car_right && ref_vehicle.lane != 2 ){
							ref_vehicle.lane++;
						} else {
							speed_diff = (car_ahead_speed - ref_vehicle.v) / (50 - (prev_size + 0.0));
							if(speed_diff < -MAX_ACC){
								speed_diff = -MAX_ACC;
							}
							MIN_SPEED = car_ahead_speed;
						}
					} else {
						if ( (!car_left && ref_vehicle.lane > 0) || (!car_right && ref_vehicle.lane != 2) ) {
							ref_vehicle.lane = 1;
						}
					}

					// Setting up target points in the future.
					vector<double> next_wp0 = getXY(ref_vehicle.s + 30, 2 + 4*ref_vehicle.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(ref_vehicle.s + 60, 2 + 4*ref_vehicle.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(ref_vehicle.s + 90, 2 + 4*ref_vehicle.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);
					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					// Making coordinates to local car coordinates.
					for (int i = 0; i < ptsx.size(); i++ ) {
						double shift_x = ptsx[i] - ref_vehicle.x;
						double shift_y = ptsy[i] - ref_vehicle.y;

						ptsx[i] = shift_x * cos(0 - ref_vehicle.yaw) - shift_y * sin(0 - ref_vehicle.yaw);
						ptsy[i] = shift_x * sin(0 - ref_vehicle.yaw) + shift_y * cos(0 - ref_vehicle.yaw);
					}

					// Create the spline.
					tk::spline s;
					s.set_points(ptsx, ptsy);

					// Output path points from previous path for continuity.
					vector<double> next_x_vals;
					vector<double> next_y_vals;
					for ( int i = 0; i < prev_size; i++ ) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}
					
					// Calculate distance y position on 30 m ahead.
					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

					for( int i = 1; i < 50 - prev_size; i++ ) {
						ref_vehicle.v += speed_diff;
						if ( ref_vehicle.v > MAX_SPEED ) {
							ref_vehicle.v = MAX_SPEED;
						} else if(ref_vehicle.v < MIN_SPEED){
							ref_vehicle.v = MIN_SPEED;
						}
						double N = target_dist/(0.02*ref_vehicle.v/2.24);
						double x_point = x_add_on + target_x/N;
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						x_point = x_ref * cos(ref_vehicle.yaw) - y_ref * sin(ref_vehicle.yaw);
						y_point = x_ref * sin(ref_vehicle.yaw) + y_ref * cos(ref_vehicle.yaw);

						x_point += ref_vehicle.x;
						y_point += ref_vehicle.y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}

					ref_vel = ref_vehicle.v;

					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
