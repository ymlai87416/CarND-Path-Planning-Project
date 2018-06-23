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
#include "utility.h"
#include "assert.h"

#include "modules/Constants.h"
#include "modules/map/map.h"
#include "modules/prediction/classifier.h"
#include "modules/localization/localization.h"
#include "modules/sensor_fusion/sensor_fusion.h"
#include "modules/prediction/prediction.h"
#include "modules/planning/planning.h"


#define ncurses_no

#ifdef ncurses
#include <ncurses.h>
#endif

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

vector<string> state_rank = {"KL", "PLCL", "PLCR", "LCL", "LCR"};
bool trajectory_cost_by_state(TrajectoryCost a, TrajectoryCost b)
{
  int a_ord, b_ord;
  vector<string>::iterator it = std::find(state_rank.begin(), state_rank.end(), a.state);
  a_ord = std::distance(state_rank.begin(), it);
  it = std::find(state_rank.begin(), state_rank.end(), b.state);
  b_ord = std::distance(state_rank.begin(), it);

  return a_ord < b_ord;
}

void displayDrivingConsole(LocalizationEstimate const& estimate,
                            vector<SensorFusionMessage> const& sensor_fusion_message,
                            PlanningResult planning_result){
#ifdef ncurses
  clear();

  start_color();			/* Start color 			*/
  init_pair(1, COLOR_BLUE, COLOR_BLACK);
  init_pair(2, COLOR_WHITE, COLOR_BLUE);

  ostringstream oss;
  string line;

  attron(COLOR_PAIR(1));
  printw("Localization\n");
  attroff(COLOR_PAIR(1));

  oss << "x: " << estimate.x << " y: " << estimate.y << " yaw: " << estimate.yaw << " speed: " << estimate.speed << "\n";
  line = oss.str(); oss.str("");
  printw(line.c_str());
  oss << "s: " << estimate.s << " d: " << estimate.d << "\n";
  line = oss.str(); oss.str("");
  printw(line.c_str());

  attron(COLOR_PAIR(1));
  printw("\nSensor Fusion\n");
  attroff(COLOR_PAIR(1));

  string header = "ID\tlane\tx\t\ty\t\tvx\t\tvy\t\ts\t\td\t\ts_dot\t\td_dot\n";
  printw(header.c_str());
  for(auto const& message: sensor_fusion_message) {
    string str_message = to_string(message.id) + "\t" + to_string(message.lane) + "\t" + to_string(message.x) + "\t"
                         + to_string(message.y) + "\t" + to_string(message.vx) + "\t"
                         + to_string(message.vy) + "\t" + to_string(message.s) + "\t"
                         + to_string(message.d) + "\t" + to_string(message.s_dot) + "\t" + to_string(message.d_dot) + "\n";
    printw(str_message.c_str());
  }

  attron(COLOR_PAIR(1));
  printw("\nBehavior\n");
  attroff(COLOR_PAIR(1));

  //latest trajectory state and target lane
  oss << "Best trajectory: state=" << setw(5) << planning_result.best_trajectory.state
      << setw(8) << " lane = " << setw(3) << planning_result.best_trajectory.target_lane
      << setw(5) << " s = " << setw(8) << planning_result.best_trajectory.target_s
      << setw(5) << " v = " << setw(8) << planning_result.best_trajectory.target_v
      << setw(5) << " a = " << setw(8) << planning_result.best_trajectory.target_a << endl;
  string best_traj_line = oss.str();
  oss.str("");

  printw(best_traj_line.c_str());


  attron(COLOR_PAIR(1));
  printw("\nScoring\n");
  attroff(COLOR_PAIR(1));

  oss << setiosflags(ios::left)
      << setw(8) << "State"<< setw(8)  << "Total"<< setw(8)  << "Sub 1"
      << setw(8) << "Sub 2" << setw(8) << "Sub 3"
      << setw(8) << "Sub 4" << setw(8) << "Sub 5" << setw(8) << "Sub 6" << setw(8) << "Sub 7" << endl;
  header = oss.str(); oss.str("");
  //header = "State\tTotal\tSub 1\tSub 2\tSub 3\tSub 4\tSub 5\tSub 6\n";
  printw(header.c_str());
  std::sort(planning_result.cost.begin(), planning_result.cost.end(), trajectory_cost_by_state);
  for(auto it = planning_result.cost.begin(); it != planning_result.cost.end(); ++it){
    if(it->sub_score.size() < 7) continue;

    oss << std::setprecision(4) << setiosflags(ios::left) << setw(8) << it->state << setw(8) << it->total_score
        << setw(8) << it->sub_score[0] << setw(8) << it->sub_score[1]
        << setw(8) << it->sub_score[2] << setw(8) << it->sub_score[3]
        << setw(8) << it->sub_score[4] << setw(8) << it->sub_score[5] << setw(8) << it->sub_score[6] << endl;
    line = oss.str(); oss.str("");
    printw(line.c_str());
  }
  /*
  */
  refresh();
#else
  cout << "Localization" << endl;
  cout << "x: " << estimate.x << " y: " << estimate.y << " yaw: " << estimate.yaw << " speed: " << estimate.speed << endl;
  cout << "s: " << estimate.s << " d: " << estimate.d << endl;

  cout << "Sensor Fusion" << endl;
  string header = "ID\tlane\tx\t\ty\t\tvx\t\tvy\t\ts\t\td\t\ts_dot\t\td_dot\n";
  cout << header.c_str() << endl;
  for(auto const& message: sensor_fusion_message) {
    string str_message = to_string(message.id) + "\t" + to_string(message.lane) + "\t" + to_string(message.x) + "\t"
                         + to_string(message.y) + "\t" + to_string(message.vx) + "\t"
                         + to_string(message.vy) + "\t" + to_string(message.s) + "\t"
                         + to_string(message.d) + "\t" + to_string(message.s_dot) + "\t" + to_string(message.d_dot) + "\n";
    cout << str_message.c_str() << endl;
  }

  cout << "Behavior" << endl;
  //latest trajectory state and target lane
  cout << "Best trajectory: state=" << setw(5) << planning_result.best_trajectory.state
      << setw(3) << planning_result.best_trajectory.target_lane
      << setw(5) << " s = " << setw(8) << planning_result.best_trajectory.target_s
      << setw(5) << " v = " << setw(8) << planning_result.best_trajectory.target_v
      << setw(5) << " a = " << setw(8) << planning_result.best_trajectory.target_a << endl;

  cout << "Scoring" << endl;
  header = "State\tTotal cost\tSub 1\tSub 2\tSub 3\tSub 4\tSub 5\tSub 6\n";
  cout << header.c_str();
  std::sort(planning_result.cost.begin(), planning_result.cost.end(), trajectory_cost_by_state);
  for(auto it = planning_result.cost.begin(); it != planning_result.cost.end(); ++it){
    if(it->sub_score.size() < 6) continue;
    cout << std::setprecision( 4 ) << it->state << "\t" << it->total_score << "\t" << it->sub_score[0] << "\t"
        << it->sub_score[1] << "\t" << it->sub_score[2] << "\t" << it->sub_score[3]
        << "\t" << it->sub_score[4] << "\t" << it->sub_score[5] << endl;
  }
#endif
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
          local_estimate.relative_time = 0;

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

          planning.OnLocalizationUpdate(local_estimate);

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

            message.s_dot = result[2];
            message.d_dot = result[3];
            message.yaw = theta;

            message.lane = map.GetLaneByLateralCoord(message.d).lane_id;

            sensor_fusion_messages.push_back(message);
          }

          prediction.OnSensorFusionUpdate(sensor_fusion_messages);
          vector<PredictionEstimate> predictions = prediction.GetPrediction();

          //do nothing here
          planning.OnPredictionUpdate(sensor_fusion_messages, predictions);

          PlanningResult planning_result = planning.PlanTrajectory();
          Trajectory trajectory = planning_result.best_trajectory;

          //displayDrivingConsole(local_estimate, sensor_fusion_messages, planning_result);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          json msgJson;

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
          //output_debug_prediction(counter, local_estimate, possible_trajactories, predictions);

          //std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
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
