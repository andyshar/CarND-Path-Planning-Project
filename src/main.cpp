#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using namespace std;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, 
               &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // #################### my code begin ####################

          int previous_path_size = previous_path_x.size();
          if (previous_path_size > 0) {
              car_s = end_path_s;
          }
          
          bool has_car_ahead = false;
          bool has_car_left = false;
          bool has_car_right = false;
          int my_car_lane = 0;
          for (int i=0; i < sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
          
              if (d > 0 && d < 4) {
                  my_car_lane = 0;
              } else if (d > 4 && d < 8) {
                  my_car_lane = 1;
              }else if(d > 8 && d < 12) {
                  my_car_lane = 2;
              }
              
              // check car in lane
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double my_car_s = sensor_fusion[i][5];
              // If using the previous point
              my_car_s += previous_path_size * 0.02 * check_speed;
              double safe_distance = 30;

              // Check a car is ahead or not
              cout << "my_car_lane : " << my_car_lane << endl;
              if (my_car_lane == lane && my_car_s - car_s > 0 && my_car_s - car_s < safe_distance) {
                  has_car_ahead = true;
                  cout << "A car ahead !" << endl;
              } else if (lane - my_car_lane > 0 && (car_s - my_car_s < safe_distance && car_s - my_car_s > -safe_distance)) {
                  has_car_left = true;
                  cout << "A car at left !" << endl;
              } else if (lane - my_car_lane < 0 && (car_s - my_car_s < safe_distance && car_s - my_car_s > -safe_distance)) {
                  has_car_right = true;
                  cout << "A car at right !" << endl;
              }
                
          }

          
          if (has_car_ahead) {
              if (!has_car_left && lane > 0) {
                  lane--;
                  cout << "Lane change : " << lane << endl;
              } else if (!has_car_right && lane < 2) {
                  lane++;
                  cout << "Lane change : " << lane << endl;
              } else {
                  ref_vel -= 0.32;
              }
          } else if ( ref_vel < 49.5) {
              ref_vel += 0.32;
              cout << "Ref vel : " << ref_vel << endl;
          }

          // #################### my code end ####################

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // #################### my code begin ####################

          vector<double> points_x;
          vector<double> points_y;
            
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
            
          // Check previous points
          if ( previous_path_size < 2 ) {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);
              
              points_x.push_back(prev_car_x);
              points_x.push_back(car_x);
              
              points_y.push_back(prev_car_y);
              points_y.push_back(car_y);
          } else {
              // Use the last two points.
              ref_x = previous_path_x[previous_path_size - 1];
              ref_y = previous_path_y[previous_path_size - 1];
              
              double ref_x_prev = previous_path_x[previous_path_size - 2];
              double ref_y_prev = previous_path_y[previous_path_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
              
              points_x.push_back(ref_x_prev);
              points_x.push_back(ref_x);
              
              points_y.push_back(ref_y_prev);
              points_y.push_back(ref_y);
          }
            
          // Set up target points in the future.
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          points_x.push_back(next_wp0[0]);
          points_x.push_back(next_wp1[0]);
          points_x.push_back(next_wp2[0]);
          
          points_y.push_back(next_wp0[1]);
          points_y.push_back(next_wp1[1]);
          points_y.push_back(next_wp2[1]);
            
          // Make coordinates to local car coordinates.
          for ( int i = 0; i < points_x.size(); i++ ) {
              double shift_x = points_x[i] - ref_x;
              double shift_y = points_y[i] - ref_y;
              
              points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }
          
          // Create a spline
          
          tk::spline s;
          s.set_points(points_x, points_y);
          
          // Output path points from previous path for continuity.
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for ( int i = 0; i < previous_path_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate distance y position on 30 meters ahead.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
          
          for (int i=0; i < 50 - previous_path_size; i++) {
              
              double N = target_dist / (0.02 * ref_vel / 3.2);
              double x_point = x_add_on + (target_x)/N;
              double y_point = s(x_point);
              
              x_add_on = x_point;
              double x_ref = x_point;
              double y_ref = y_point;
              // Rotate back to normal after it earlier
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
                
          }

          // #################### my code end ####################

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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