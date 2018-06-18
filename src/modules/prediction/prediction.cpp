//
// Created by tom on 6/11/18.
//

#include "prediction.h"
#include "../Constants.h"
#include "../../spline.h"
#include "../../utility.h"

Prediction::Prediction(Map const* map)
{
  this-> map = map;
}

Prediction::~Prediction()
{

}

//In the current implementation, I assume that all vehicle is moving forward with ds/dt
bool Prediction::PredictVehiclesTrajectory(vector<SensorFusionMessage> vehicle_list,
                                           vector<PredictionEstimate>& trajectory_list){

  float vehicle_s[3];
  for(SensorFusionMessage const& vehicle: vehicle_list){

    PredictionEstimate estimate;
    Lane lane = map->GetLane(vehicle.lane);

    estimate.id = vehicle.id;
    estimate.probabilty = 1;
    estimate.trajectory.state = "Constant speed";
    estimate.trajectory.total_path_length = vehicle.s_dot * prediction_time_length;
    estimate.trajectory.total_path_time = prediction_time_length;

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = vehicle.x;
    double ref_y = vehicle.y;
    double ref_yaw = vehicle.yaw;
    double ref_speed = vehicle.s_dot;

    if(fabs(ref_speed) > 1e-6) {
      ptsx.push_back(vehicle.x);
      ptsy.push_back(vehicle.y);
      for (int i = 1; i <= spline_total_step; ++i) {
        vehicle_s[i] = vehicle.s + vehicle.s_dot * spline_time_step * i;
        vector<double> next_wp = map->GetXY(vehicle_s[i], lane.lane_center_d);
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
      }

      for (int i = 0; i < ptsx.size(); ++i) {
        double x_dash, y_dash;
        ConvertXYToVehicleCoordination(ptsx[i], ptsy[i], ref_x, ref_y, ref_yaw, x_dash, y_dash);

        ptsx[i] = x_dash;
        ptsy[i] = y_dash;
      }

      tk::spline s;

      s.set_points(ptsx, ptsy);

      double target_x = vehicle.s_dot * prediction_time_length;
      double target_y = s(target_x);
      double target_dist = sqrt(target_x * target_x + target_y * target_y);
      double x_add_on = 0;
      float cur_time = 0;

      double x_point, y_point, x_point_next, y_point_next;
      double s_, d_, s_next, d_next;
      x_point_next = vehicle.x;
      y_point_next = vehicle.y;
      vector<double> frenet_coord = map->GetFrenet(vehicle.x, vehicle.y, vehicle.yaw);
      s_next = frenet_coord[0];
      d_next = frenet_coord[1];

      double N = (target_dist / trajectory_discretize_timestep / vehicle.s_dot);

      //the trajectory must be start at time = 0.02;
      int step = ceil(prediction_time_length / trajectory_discretize_timestep);

      for (int i = 1; i <= step; ++i) {
        x_point = x_point_next;
        y_point = y_point_next;
        s_ = s_next;
        d_ = d_next;
        cur_time = trajectory_discretize_timestep * i;

        double x_dash_next, y_dash_next;
        x_dash_next = x_add_on + (target_x) / N;
        y_dash_next = s(x_dash_next);

        x_add_on = x_dash_next;

        ConvertVehicleCoordinateToXY(x_dash_next, y_dash_next, ref_x, ref_y, ref_yaw, x_point_next, y_point_next);

        PathPoint path_point;
        path_point.x = x_point_next;
        path_point.y = y_point_next;
        path_point.s_dot = ref_speed;
        path_point.d_dot = 0;
        path_point.relative_time = cur_time;

        estimate.trajectory.path_point_list.push_back(path_point);
      }

      for (int i = 0; i < estimate.trajectory.path_point_list.size() - 1; ++i) {
        double dx = estimate.trajectory.path_point_list[i + 1].x - estimate.trajectory.path_point_list[i].x;
        double dy = estimate.trajectory.path_point_list[i + 1].y - estimate.trajectory.path_point_list[i].y;
        double yaw = atan2(dy, dx);
        estimate.trajectory.path_point_list[i].theta = yaw;
        frenet_coord = map->GetFrenet(estimate.trajectory.path_point_list[i].x,
                                      estimate.trajectory.path_point_list[i].y, estimate.trajectory.path_point_list[i].theta);
        estimate.trajectory.path_point_list[i].s = frenet_coord[0];
        estimate.trajectory.path_point_list[i].d = frenet_coord[1];
        estimate.trajectory.path_point_list[i].vx = estimate.trajectory.path_point_list[i].s_dot * cos(yaw);
        estimate.trajectory.path_point_list[i].vy = estimate.trajectory.path_point_list[i].s_dot * sin(yaw);
      }
    }
    else{
      int step = ceil(prediction_time_length / trajectory_discretize_timestep);

      for (int i = 1; i <= step; ++i) {
        PathPoint path_point;
        path_point.x = vehicle.x;
        path_point.y = vehicle.y;
        path_point.relative_time = trajectory_discretize_timestep * i;
        estimate.trajectory.path_point_list.push_back(path_point);
      }
    }

    trajectory_list.push_back(estimate);
  }

  return true;
}

void Prediction::initClassifier() {
  vector<string> possible_vector = {"CLL","KL","CLR"};

  vector<double> piror = {0.285, 0.421, 0.293};

  vector<vector<double>> mean_list;
  mean_list.push_back({19.714, 20.324, 19.477});
  mean_list.push_back({5.052, 3.680, 2.934});
  mean_list.push_back({9.914, 9.999, 9.947});
  mean_list.push_back({-0.967, 0.006, 0.954});

  vector<vector<double>> std_dev_list;
  std_dev_list.push_back({12.290, 11.436, 12.085});
  std_dev_list.push_back({2.360, 3.404, 2.312});
  std_dev_list.push_back({0.990, 1.069, 0.952});
  std_dev_list.push_back({0.663, 0.168, 0.647});

  state_classifier.Load(possible_vector, mean_list, std_dev_list, piror);
}