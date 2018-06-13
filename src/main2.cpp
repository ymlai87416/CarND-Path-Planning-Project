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
#include "map.h"
#include "utility.h"
#include <ncurses.h>
#include "classifier.h"

#define ncurses

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

void output_sensor_fusion(vector<vector<double>> sensor_fusion, Map& map, GNB& classifier){
  /* structure of the sensor fusion input
   *  id    int,
   *  x     float,
   *  y     float,
   *  vx    float,
   *  vy    float,
   *  s     float,
   *  d     float
   */

#ifdef ncurses
  clear();
  string header = "ID\tlane\tx\t\ty\t\tvx\t\tvy\t\ts\t\td\t\ts_dot\t\td_dot\t\tPrediction\n";
  printw(header.c_str());
  for(int i=0; i<sensor_fusion.size(); ++i) {
    float theta = atan2(sensor_fusion[i][4], sensor_fusion[i][3]);
    float speed = sqrt(sensor_fusion[i][4]*sensor_fusion[i][4] + sensor_fusion[i][3]*sensor_fusion[i][3]);

    vector<double> result = map.getFrenet(sensor_fusion[i][1], sensor_fusion[i][2], theta, speed);
    float s_dot = result[2];
    float d_dot = result[3];

    int lane = float(sensor_fusion[i][6] / 4);
    string pred = classifier.predict({0, sensor_fusion[i][6], s_dot, d_dot});

    string message = to_string((int)sensor_fusion[i][0]) + "\t" + to_string(lane) + "\t" + to_string(sensor_fusion[i][1]) + "\t"
                     + to_string(sensor_fusion[i][2]) + "\t" + to_string(sensor_fusion[i][3]) + "\t"
                     + to_string(sensor_fusion[i][4]) + "\t" + to_string(sensor_fusion[i][5]) + "\t"
                     + to_string(sensor_fusion[i][6]) + "\t" + to_string(s_dot) + "\t" + to_string(d_dot)
                     + "\t" + pred + "\n";
    printw(message.c_str());
  }
  refresh();
#endif
}

void init_classifier(GNB& classifier) {
  vector<string> possible_vector = {"CLL","KL","CLR"};

  vector<double> piror = {0.285, 0.421, 0.293};

  vector<vector<double>> mean_list;
  mean_list.push_back({19.714, 5.052, 9.914, -0.967});
  mean_list.push_back({20.324, 3.680, 9.999, 0.006});
  mean_list.push_back({19.477, 2.934, 9.947, 0.954});

  vector<vector<double>> std_dev_list;
  std_dev_list.push_back({12.290, 2.360, 0.990, 0.663});
  std_dev_list.push_back({11.436, 3.404, 1.069, 0.168});
  std_dev_list.push_back({12.085, 2.312, 0.952, 0.647});

  classifier.load(possible_vector, mean_list, std_dev_list, piror);
}

int main() {
  uWS::Hub h;

#ifdef ncurses
  initscr();
  refresh();
#endif

  string map_file_ = "../data/highway_map.csv";
  Map map(map_file_);
  GNB classifier;
  init_classifier(classifier);

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  int lane = 1;
  double ref_vel = 0;

  h.onMessage([&map, &lane, &ref_vel, &classifier](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion_raw = j[1]["sensor_fusion"];

          vector<vector<double>> sensor_fusion;
          for(int i=0; i<sensor_fusion_raw.size(); ++i){
            vector<double> sensor_fusion_row;
            for(int j=0; j<7; ++j)
              sensor_fusion_row.push_back(sensor_fusion_raw[i][j]);

            sensor_fusion.push_back(sensor_fusion_row);
          }

          output_sensor_fusion(sensor_fusion, map, classifier);

          int prev_size = previous_path_x.size();

          if(prev_size > 0){
            car_s = end_path_s;
          }

          bool too_close = false;

          //find ref_v to use
          for(int i=0; i<sensor_fusion.size(); ++i){
            //car is in my lane
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d>(2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s+=((double) prev_size*.02* check_speed); //if using previous points can project s value out

              //check s values greater than mine and s gap
              if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
              {

                //Do some logic here, lower reference velocity so we don't crash into the car infront of us
                //also flag to try to change lanes.
                //ref_vel = 29.5;

                too_close= true;
                if(lane > 0)
                  lane = 0;
              }
            }
          }

          if(too_close){
            ref_vel -= .224;
          }
          else if (ref_vel < 49.5){
            ref_vel += .224;
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
            vector<double> xy=getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }*/

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double> next_wp0 = map.getXY(car_s + 30, (2+4*lane));
          vector<double> next_wp1 = map.getXY(car_s + 60, (2+4*lane));
          vector<double> next_wp2 = map.getXY(car_s + 90, (2+4*lane));

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for(int i=0; i<ptsx.size(); ++i){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          for(int i=0; i<previous_path_x.size(); ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;

          for(int i=1; i<=50-previous_path_x.size(); ++i){
            double N = (target_dist / .02 / ref_vel * 2.24);
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //car -> global coordination
            x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

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
