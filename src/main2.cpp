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
#include "spline.h"
#include "modules/map/map.h"
#include "utility.h"
#include <ncurses.h>
#include "modules/prediction/classifier.h"
#include "modules/localization/localization.h"
#include "modules/sensor_fusion/sensor_fusion.h"
#include "assert.h"
#include "modules/Constants.h"
#include "modules/prediction/prediction.h"
#include "modules/planning/planning.h"

#define ncursesx

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


void output_sensor_fusion(vector<SensorFusionMessage> sensor_fusion_msgs, GNB& classifier){
#ifdef ncurses
  clear();
  string header = "ID\tlane\tx\t\ty\t\tvx\t\tvy\t\ts\t\td\t\ts_dot\t\td_dot\t\tPrediction\n";
  printw(header.c_str());
  for(auto const& message: sensor_fusion_msgs) {

    string pred = classifier.Predict({0, message.d, message.s_dot, message.d_dot});

    string str_message = to_string(message.id) + "\t" + to_string(message.lane) + "\t" + to_string(message.x) + "\t"
                     + to_string(message.y) + "\t" + to_string(message.vx) + "\t"
                     + to_string(message.vy) + "\t" + to_string(message.s) + "\t"
                     + to_string(message.d) + "\t" + to_string(message.s_dot) + "\t" + to_string(message.d_dot)
                     + "\t" + pred + "\n";
    printw(str_message.c_str());
  }
  refresh();
#endif
}

void output_debug_prediction(int counter, LocalizationEstimate estimate,
                             vector<double> next_val_x,
                             vector<double> next_val_y,
                             vector<PredictionEstimate> prediction){
  string output_dir= "/home/tom/GitProjects/CarND-Path-Planning-Project/debug/";
  string filename;
  stringstream ss;
  ss << setfill('0') << setw(5) << counter << ".txt";
  ss >> filename;
  ofstream debug_file;
  debug_file.open(output_dir + filename);

  int step = ceil(planning_time_length / trajectory_discretize_timestep);

  for(int i=0; i<step; ++i){
    debug_file << next_val_x[i] - estimate.x << "\t" << next_val_y[i] - estimate.y << "\t";
    for(int j=0; j<prediction.size(); ++j){
      debug_file << prediction[j].trajectory.path_point_list[i].x - estimate.x << "\t"
                 << prediction[j].trajectory.path_point_list[i].y - estimate.y << "\t";
    }
    debug_file << "\n";
  }

  debug_file.close();
}

void output_debug_prediction(int counter, LocalizationEstimate estimate,
                             Trajectory trajectory,
                             vector<PredictionEstimate> prediction){
  string output_dir= "/home/tom/GitProjects/CarND-Path-Planning-Project/debug/";
  string filename;
  stringstream ss;
  ss << setfill('0') << setw(5) << counter << ".txt";
  ss >> filename;
  ofstream debug_file;
  debug_file.open(output_dir + filename);

  int step = ceil(planning_time_length / trajectory_discretize_timestep);

  for(int i=0; i<step; ++i){
    debug_file << trajectory.path_point_list[i].x - estimate.x << "\t" << trajectory.path_point_list[i].y - estimate.y << "\t";
    for(int j=0; j<prediction.size(); ++j){
      debug_file << prediction[j].trajectory.path_point_list[i].x - estimate.x << "\t"
                 << prediction[j].trajectory.path_point_list[i].y - estimate.y << "\t";
    }
    debug_file << "\n";
  }

  debug_file.close();
}

int main() {
  uWS::Hub h;

#ifdef ncurses
  initscr();
  refresh();
#endif

  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  Map map(map_file_, max_s, 3);
  Prediction prediction(&map);
  Planning planning(&map);

  int lane_id = 1;
  double ref_vel = 0;
  int counter = 0;

  h.onMessage([&map, &lane_id, &ref_vel, &prediction, &planning, &counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      std::clock_t start;
      start = std::clock();
      counter++;

      auto s = hasData(data);
      Lane lane = map.GetLane(lane_id);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          LocalizationEstimate local_estimate;

        	// Main car's localization Data
          local_estimate.x = j[1]["x"];
          local_estimate.y = j[1]["y"];
          local_estimate.s = j[1]["s"];
          local_estimate.d = j[1]["d"];
          local_estimate.yaw = j[1]["yaw"];
          local_estimate.speed = j[1]["speed"];

          // Previous path data given to the Planner
          local_estimate.previous_path_x.clear();
          local_estimate.previous_path_y.clear();
          for(auto d : j[1]["previous_path_x"])
            local_estimate.previous_path_x.push_back(d);
          for(auto d: j[1]["previous_path_y"])
            local_estimate.previous_path_y.push_back(d);

          // Previous path's end s and d values
          local_estimate.end_path_s = j[1]["end_path_s"];
          local_estimate.end_path_d = j[1]["end_path_d"];

          if(local_estimate.previous_path_x.size() >= 2) {
            //finish the calculation properties
            vector<double> next_frenet_0 = map.GetFrenet(local_estimate.previous_path_x[0],
                                                         local_estimate.previous_path_y[0],
                                                         atan2(local_estimate.previous_path_y[1] -
                                                               local_estimate.previous_path_y[0],
                                                               local_estimate.previous_path_x[1] -
                                                               local_estimate.previous_path_x[0]));
            vector<double> next_frenet_1 = map.GetFrenet(local_estimate.previous_path_x[1],
                                                         local_estimate.previous_path_y[1],
                                                         atan2(local_estimate.previous_path_y[2] -
                                                               local_estimate.previous_path_y[1],
                                                               local_estimate.previous_path_x[2] -
                                                               local_estimate.previous_path_x[1]));

            float vx_t0 = (local_estimate.previous_path_x[0] - local_estimate.x) / trajectory_discretize_timestep;
            float vx_t1 = (local_estimate.previous_path_x[1] - local_estimate.previous_path_x[0]) / trajectory_discretize_timestep;

            float vy_t0 = (local_estimate.previous_path_y[0] - local_estimate.y) / trajectory_discretize_timestep;;
            float vy_t1 = (local_estimate.previous_path_y[1] - local_estimate.previous_path_y[0]) / trajectory_discretize_timestep;;

            float s_dot_t0 = (next_frenet_0[0] - local_estimate.s) / trajectory_discretize_timestep;
            float s_dot_t1 = (next_frenet_1[0] - next_frenet_0[0]) / trajectory_discretize_timestep;

            float d_dot_t0 = (next_frenet_0[1] - local_estimate.d) / trajectory_discretize_timestep;
            float d_dot_t1 = (next_frenet_1[1] - next_frenet_0[1]) / trajectory_discretize_timestep;

            local_estimate.ax = (vx_t1 - vx_t0) / trajectory_discretize_timestep;
            local_estimate.ay = (vy_t1 - vy_t0) / trajectory_discretize_timestep;

            local_estimate.s_dot = s_dot_t0;
            local_estimate.d_dot = d_dot_t0;
            local_estimate.s_dot2 = (s_dot_t1 - s_dot_t0) / trajectory_discretize_timestep;
            local_estimate.d_dot2 = (d_dot_t1 - d_dot_t0) / trajectory_discretize_timestep;
          }
          else{
            local_estimate.ax = 0;
            local_estimate.ay = 0;
            local_estimate.s_dot = 0;
            local_estimate.d_dot = 0;
            local_estimate.s_dot2 = 0;
            local_estimate.d_dot2 = 0;
          }


          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion_raw = j[1]["sensor_fusion"];

          vector<SensorFusionMessage> sensor_fusion_messages;
          for(int i=0; i<sensor_fusion_raw.size(); ++i){
            SensorFusionMessage message;

            message.id = sensor_fusion_raw[i][0];
            message.x = sensor_fusion_raw[i][1];
            message.y = sensor_fusion_raw[i][2];
            message.vx = sensor_fusion_raw[i][3];
            message.vy = sensor_fusion_raw[i][4];
            message.s = sensor_fusion_raw[i][5];
            message.d = sensor_fusion_raw[i][6];

            float theta = atan2(message.vy, message.vx);
            float speed = sqrt(message.vy*message.vy + message.vx*message.vx);

            //In this simple model, s_dot should be ~= speed and d_dot should be = 0, as cars don't change lane.
            vector<double> result = map.GetFrenet(message.x, message.y, theta, speed);

            float sanity_s = fabs(message.s - result[0]) / message.s;
            float sanity_d = fabs(message.d - result[1]) / message.d;

            /*
            if(sanity_s > 1e-1 || sanity_d > 1e-1)
              cout << "Sanity check ?? " << sanity_s << " " << sanity_d << endl;
              */

            message.s_dot = result[2];
            message.d_dot = result[3];
            message.yaw = theta;

            message.lane = message.d / 4;

            sensor_fusion_messages.push_back(message);
          }

          vector<PredictionEstimate> predictions;
          prediction.PredictVehiclesTrajectory(sensor_fusion_messages, predictions);

          //do nothing here
          planning.UpdateLocalization(local_estimate);
          planning.UpdatePrediction(sensor_fusion_messages, predictions);

          Trajectory trajectory = planning.PlanTrajectory();

          int prev_size = local_estimate.previous_path_x.size();

          double car_s = local_estimate.s;

          if(prev_size > 0){
            car_s = local_estimate.end_path_s;
          }

          bool too_close = false;

          //find ref_v to use
          for(auto const& message: sensor_fusion_messages){
            //car is in my lane
            float d = message.d;
            if(d < lane.lane_right_d && d>lane.lane_left_d){
              double check_speed = sqrt(message.vx*message.vx+message.vy*message.vy);
              double check_car_s = message.s;

              check_car_s+=((double) prev_size* trajectory_discretize_timestep * check_speed); //if using previous points can project s value out

              //check s values greater than mine and s gap
              if((check_car_s > car_s) && ((check_car_s - car_s) < planning_safe_follow_distance))
              {

                //Do some logic here, lower reference velocity so we don't crash into the car infront of us
                //also flag to try to change lanes.
                //ref_vel = 29.5;

                too_close= true;
                if(lane_id > 0)
                  lane_id = 0;
              }
            }
          }

          if(too_close){
            ref_vel -= vehicle_normal_acceleration * trajectory_discretize_timestep;
          }
          else if (ref_vel < vehicle_speed_limit){
            ref_vel += vehicle_normal_acceleration * trajectory_discretize_timestep;
          }

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          /* Demo 1 - make the car move
          double dist_inc = 0.3;
          for(int i=0; i<50; ++i) {
            double next_s = car_s + (i+1)*dist_inc;
            double next_d = 6;
            vector<double> xy=GetXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }*/

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = local_estimate.x;
          double ref_y = local_estimate.y;
          double ref_yaw = deg2rad(local_estimate.yaw);

          if(prev_size < 2){
            double prev_car_x = local_estimate.x - cos(local_estimate.yaw);
            double prev_car_y = local_estimate.y - sin(local_estimate.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(local_estimate.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(local_estimate.y);
          }
          else{
            ref_x = local_estimate.previous_path_x[prev_size-1];
            ref_y = local_estimate.previous_path_y[prev_size-1];

            double ref_x_prev = local_estimate.previous_path_x[prev_size-2];
            double ref_y_prev = local_estimate.previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = map.GetXY(car_s + 30, lane.lane_center_d);
          vector<double> next_wp1 = map.GetXY(car_s + 60, lane.lane_center_d);
          vector<double> next_wp2 = map.GetXY(car_s + 90, lane.lane_center_d);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i=0; i<ptsx.size(); ++i){
            /*
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            */

            double x_dash, y_dash;

            ConvertXYToVehicleCoordination(ptsx[i], ptsy[i], ref_x, ref_y, ref_yaw, x_dash, y_dash);

            ptsx[i] = x_dash;
            ptsy[i] = y_dash;

          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          for(int i=0; i<local_estimate.previous_path_x.size(); ++i){
            next_x_vals.push_back(local_estimate.previous_path_x[i]);
            next_y_vals.push_back(local_estimate.previous_path_y[i]);
          }

          double target_x = ref_vel * 1.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;


          for(int i=1; i<=50-local_estimate.previous_path_x.size(); ++i){
            double N = (target_dist / trajectory_discretize_timestep / ref_vel);

            double x_point, y_point, x_dash, y_dash;

            x_dash = x_add_on + (target_x)/N;
            y_dash = s(x_dash);
            x_add_on = x_dash;

            ConvertVehicleCoordinateToXY(x_dash, y_dash, ref_x, ref_y, ref_yaw, x_point, y_point);

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          //debug trajectory start
          next_x_vals.clear();
          next_y_vals.clear();

          for(int i=0; i<trajectory.path_point_list.size(); ++i){
            next_x_vals.push_back(trajectory.path_point_list[i].x);
            next_y_vals.push_back(trajectory.path_point_list[i].y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          //debug prediction
          //output_debug_prediction(counter, local_estimate, trajectory, predictions);

          std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
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
#ifdef ncurses
    printw("Connected!!!");
#else
    std::cout << "Connected!!!" << std::endl;
#endif
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
#ifdef ncurses
    endwin();
#else
    std::cout << "Disconnected" << std::endl;
#endif
  });

  int port = 4567;
  if (h.listen(port)) {
#ifdef ncurses
    string message = "Listening to port " + std::to_string(port);
    clear();
    printw(message.c_str());
    refresh();
#else
    std::cout << "Listening to port " << port << std::endl;
#endif
  } else {
#ifdef ncurses
    endwin();
#else
    std::cerr << "Failed to listen to port" << std::endl;
#endif
    return -1;
  }
  h.run();
}
