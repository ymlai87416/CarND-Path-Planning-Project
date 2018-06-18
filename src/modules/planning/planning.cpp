//
// Created by tom on 6/13/18.
//

#include "planning.h"
#include "../Constants.h"
#include <sstream>
#include "../../Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Planning::Planning(Map const* map)
{
  this-> map = map;
  this->state = "KL";
  this->target_lane = 1;
}

Planning::~Planning()
{

}

void Planning::UpdateLocalization(LocalizationEstimate estimate)
{
  this->local_estimate_snapshot = estimate;
}

void Planning::UpdatePrediction(vector<SensorFusionMessage> vehicle_list, vector<PredictionEstimate> vehicle_trajectories)
{
  this->vehicle_list_snapshot = vehicle_list;
  this->predict_trajectory_snapshot = vehicle_trajectories;
}

Trajectory Planning::PlanTrajectory()
{
  Trajectory result;

  LocalizationEstimate estimate = this->local_estimate_snapshot;
  vector<SensorFusionMessage> vehicle_list = this->vehicle_list_snapshot;
  vector<PredictionEstimate> predictions = this->predict_trajectory_snapshot;

  vector<string> states = GetSuccessorStates(estimate);
  float cost;
  vector<float> costs;
  vector<string> final_states;
  vector<Trajectory> final_trajectories;

  //convert CLL / CLR to KL if the lane change is successful
  if(target_lane == map->GetLaneByLateralCoord(estimate.d).lane_id)
    state = "KL";

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    Trajectory trajectory = GenerateTrajectory(*it, estimate, vehicle_list);
    if (trajectory.path_point_list.size() != 0) {
      cost = CalculateCost(trajectory, predictions);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  //convert to next state
  state = final_trajectories[best_idx].state;
  target_lane = final_trajectories[best_idx].target_lane;

  return final_trajectories[best_idx];
}

vector<float> Planning::GetKinematics(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list, int lane)
{
  float max_velocity_accel_limit = vehicle_max_acceleration * planning_time_length + estimate.speed;
  float new_position;
  float new_velocity;
  float new_accel;
  SensorFusionMessage vehicle_ahead;
  SensorFusionMessage vehicle_behind;

  if (GetVehicleAhead(estimate, vehicle_list, lane, vehicle_ahead)) {

    if (GetVehicleBehind(estimate, vehicle_list, lane, vehicle_behind)) {
      new_velocity = vehicle_ahead.s_dot; //must travel at the speed of traffic, regardless of preferred buffer
    } else {
      float max_velocity_in_front = (vehicle_ahead.s - estimate.s - planning_safe_follow_distance) + vehicle_ahead.s_dot - 0.5 * (0); //? acceleration?
      new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), vehicle_speed_limit);
    }
  } else {
    new_velocity = min(max_velocity_accel_limit, vehicle_speed_limit);
  }

  new_accel = new_velocity - estimate.speed; //Equation: (v_1 - v_0)/t = acceleration
  new_position = estimate.s + new_velocity + new_accel / 2.0;
  return{new_position, new_velocity, new_accel};
}

bool Planning::FeasibilityCheck(Trajectory trajectory)
{
  return false;
}

Trajectory Planning::GenerateTrajectory(string state, LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list)
{
  Trajectory trajectory;
  if (state.compare("KL") == 0) {
    trajectory = KeepLaneTrajectory(estimate, vehicle_list);
  } else if (state.compare("LCL") == 0){
    trajectory = LaneChangeTrajectory(estimate, vehicle_list, state, map->GetLaneByLateralCoord(estimate.d).lane_id-1);
  } else if (state.compare("LCR") == 0) {
    trajectory = LaneChangeTrajectory(estimate, vehicle_list, state, map->GetLaneByLateralCoord(estimate.d).lane_id+1);
  } else if (state.compare("PLCL") == 0){
    trajectory = PrepLaneChangeTrajectory(estimate, vehicle_list, state, map->GetLaneByLateralCoord(estimate.d).lane_id-1);
  } else if (state.compare("PLCR") == 0) {
    trajectory = PrepLaneChangeTrajectory(estimate, vehicle_list, state, map->GetLaneByLateralCoord(estimate.d).lane_id+1);
  }
  return trajectory;
}

void Planning::FindActualVelocityAndAcceleration(vector<PathPoint>& path_point_list)
{
  for(int i=0, j=1; j<path_point_list.size(); ++i, ++j){
    path_point_list[i].vx = (path_point_list[j].x - path_point_list[i].x) / update_interval;
    path_point_list[i].vy = (path_point_list[j].y - path_point_list[i].y) / update_interval;
  }
  for(int i=0, j=1; j<path_point_list.size(); ++i, ++j){
    path_point_list[i].ax = (path_point_list[j].vx - path_point_list[i].vx) / update_interval;
    path_point_list[i].ay = (path_point_list[j].vy - path_point_list[i].vy) / update_interval;
  }
}

Trajectory Planning::KeepLaneTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list)
{
  Trajectory trajectory;
  std::ostringstream stringStream;
  Lane current_lane = map->GetLaneByLateralCoord(estimate.d);
  stringStream << "Keep Lane" << current_lane.lane_id;
  trajectory.debug_state = stringStream.str();
  trajectory.state = "KL";
  trajectory.target_lane = current_lane.lane_id;

  vector<float> kinematics = GetKinematics(estimate, vehicle_list, current_lane.lane_id);

  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];

  vector<double> longitude_jmt = Jmt({estimate.s, estimate.s_dot, 0}, {new_s, new_v, new_a}, planning_time_length);
  vector<double> latitude_jmt = Jmt({estimate.d, 0, 0}, {current_lane.lane_center_d, 0, 0}, planning_time_length);

  int num_timestep = (int)ceil(planning_time_length / trajectory_discretize_timestep);
  for(int i=1; i<=num_timestep; ++i){
    //find out s, s_dot, s_dot2, d, d_dot, d_dot2 at time t
    //find out x, y, yaw, speed, acceleration
    //the trajectory point is complete
    float rt = trajectory_discretize_timestep * i;
    PathPoint path_point;

    vector<double> long_s = Jmt_t(longitude_jmt, rt);
    vector<double> lat_s = Jmt_t(latitude_jmt, rt);

    path_point.s = long_s[0];
    path_point.d = lat_s[0];
    path_point.s_dot = long_s[1];
    path_point.s_dot2 = long_s[2];
    path_point.d_dot = lat_s[1];
    path_point.d_dot2 = lat_s[2];

    vector<double> xy_coord = map->GetXY(long_s[0], lat_s[0], long_s[1], lat_s[1]);
    path_point.x = xy_coord[0];
    path_point.y = xy_coord[1];
    path_point.theta = xy_coord[2];

    trajectory.path_point_list.push_back(path_point);
  }

  FindActualVelocityAndAcceleration(trajectory.path_point_list);

  return trajectory;
}

Trajectory Planning::LaneChangeTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list,
                                          string state, int target_lane_id)
{
  /*
  Generate a lane change trajectory.
  */

  Trajectory trajectory;
  std::ostringstream stringStream;
  Lane current_lane = map->GetLaneByLateralCoord(estimate.d);
  Lane target_lane = map->GetLane(target_lane_id);
  stringStream << "Change Lane current: " << current_lane.lane_id << " to: " << target_lane_id;
  trajectory.debug_state = stringStream.str();
  trajectory.state = state;
  trajectory.target_lane = target_lane_id;

  SensorFusionMessage next_lane_vehicle;
  //Check if a lane change is possible (check if another vehicle occupies that spot).
  for (vector<SensorFusionMessage>::const_iterator it = vehicle_list.begin(); it != vehicle_list.end(); ++it) {
    next_lane_vehicle = *it;
    if (fabs(next_lane_vehicle.s - estimate.s) < planning_safe_follow_distance && next_lane_vehicle.lane == target_lane_id) {
      //If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }

  vector<float> kinematics = GetKinematics(estimate, vehicle_list, target_lane_id);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];

  //TODO: test out if 1s is enough for a jerkless path
  vector<double> longitude_jmt = Jmt({estimate.s, estimate.s_dot, 0}, {new_s, new_v, new_a}, planning_time_length);
  vector<double> latitude_jmt = Jmt({estimate.d, 0, 0}, {target_lane.lane_center_d, 0, 0}, planning_time_length);

  int num_timestep = (int)ceil(planning_time_length / trajectory_discretize_timestep);
  for(int i=1; i<=num_timestep; ++i){
    //find out s, s_dot, s_dot2, d, d_dot, d_dot2 at time t
    //find out x, y, yaw, speed, acceleration
    //the trajectory point is complete
    float rt = trajectory_discretize_timestep * i;
    PathPoint path_point;

    vector<double> long_s = Jmt_t(longitude_jmt, rt);
    vector<double> lat_s = Jmt_t(latitude_jmt, rt);

    path_point.s = long_s[0];
    path_point.d = lat_s[0];
    path_point.s_dot = long_s[1];
    path_point.s_dot2 = long_s[2];
    path_point.d_dot = lat_s[1];
    path_point.d_dot2 = lat_s[2];

    vector<double> xy_coord = map->GetXY(long_s[0], lat_s[0], long_s[1], lat_s[1]);
    path_point.x = xy_coord[0];
    path_point.y = xy_coord[1];
    path_point.theta = xy_coord[2];

    trajectory.path_point_list.push_back(path_point);
  }

  FindActualVelocityAndAcceleration(trajectory.path_point_list);

  return trajectory;
}

Trajectory Planning::PrepLaneChangeTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list,
                                              string state, int target_lane_id)
{
  /*
  Generate a trajectory preparing for a lane change.
  */
  Trajectory trajectory;

  std::ostringstream stringStream;
  Lane target_lane = map->GetLane(target_lane_id);
  Lane current_lane = map->GetLaneByLateralCoord(estimate.d);
  stringStream << "Prep change lane: current" << current_lane.lane_id << " to: " << target_lane_id;
  trajectory.debug_state = stringStream.str();
  trajectory.state = state;
  trajectory.target_lane = target_lane_id;

  float new_s;
  float new_v;
  float new_a;
  SensorFusionMessage vehicle_behind;
  vector<float> curr_lane_new_kinematics =
      GetKinematics(estimate, vehicle_list, current_lane.lane_id);

  if (GetVehicleBehind(estimate, vehicle_list, current_lane.lane_id, vehicle_behind)) {
    //Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];

  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = GetKinematics(estimate, vehicle_list, target_lane_id);
    //Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  vector<double> longitude_jmt = Jmt({estimate.s, estimate.s_dot, 0}, {new_s, new_v, new_a}, planning_time_length);
  vector<double> latitude_jmt = Jmt({estimate.d, 0, 0}, {current_lane.lane_center_d, 0, 0}, planning_time_length);

  int num_timestep = (int)ceil(planning_time_length / trajectory_discretize_timestep);
  for(int i=1; i<=num_timestep; ++i){
    //find out s, s_dot, s_dot2, d, d_dot, d_dot2 at time t
    //find out x, y, yaw, speed, acceleration
    //the trajectory point is complete
    float rt = trajectory_discretize_timestep * i;
    PathPoint path_point;

    vector<double> long_s = Jmt_t(longitude_jmt, rt);
    vector<double> lat_s = Jmt_t(latitude_jmt, rt);

    path_point.s = long_s[0];
    path_point.d = lat_s[0];
    path_point.s_dot = long_s[1];
    path_point.s_dot2 = long_s[2];
    path_point.d_dot = lat_s[1];
    path_point.d_dot2 = lat_s[2];

    vector<double> xy_coord = map->GetXY(long_s[0], lat_s[0], long_s[1], lat_s[1]);
    path_point.x = xy_coord[0];
    path_point.y = xy_coord[1];
    path_point.theta = xy_coord[2];

    trajectory.path_point_list.push_back(path_point);
  }

  FindActualVelocityAndAcceleration(trajectory.path_point_list);

  return trajectory;
}

bool Planning::CheckForCollision(Trajectory trajectory)
{
  return false;
}

vector<string> Planning::GetSuccessorStates(LocalizationEstimate const& estimate){
  Lane lane = map->GetLaneByLateralCoord(estimate.d);

  vector<string> states;

  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("KL");
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    states.push_back("KL");
    if (lane.lane_id != map->lanes_available - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    states.push_back("KL");
    if (lane.lane_id != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0){
      states.push_back(state);
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

bool Planning::GetVehicleBehind(LocalizationEstimate const& estimate,vector<SensorFusionMessage> const& vehicle_list,
                                int target_lane, SensorFusionMessage & rVehicle){
  int max_s = -1;
  bool found_vehicle = false;
  Lane lane= map->GetLane(target_lane);

  SensorFusionMessage temp_vehicle;
  for (vector<SensorFusionMessage>::const_iterator it = vehicle_list.begin(); it != vehicle_list.end(); ++it) {
    temp_vehicle = *it;

    float temp_vehicle_s = temp_vehicle.s;
    if(fabs(temp_vehicle_s - estimate.s) > sensor_fusion_range) {
      double a = temp_vehicle.s + map->max_s - estimate.s;
      double b = estimate.s - (temp_vehicle.s - map->max_s);

      if(a < sensor_fusion_range)
        temp_vehicle_s = temp_vehicle.s + map->max_s;
      else
        temp_vehicle_s = temp_vehicle.s - map->max_s;
    }

    if (temp_vehicle.lane == lane.lane_id && temp_vehicle_s < estimate.s && temp_vehicle_s > max_s) {
      max_s = temp_vehicle_s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

bool Planning::GetVehicleAhead(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list,
                               int target_lane, SensorFusionMessage & rVehicle){

  int min_s = 99999999;
  bool found_vehicle = false;
  Lane lane=  map->GetLane(target_lane);

  SensorFusionMessage temp_vehicle;
  for (vector<SensorFusionMessage>::const_iterator it = vehicle_list.begin(); it != vehicle_list.end(); ++it) {
    temp_vehicle = *it;

    float temp_vehicle_s = temp_vehicle.s;
    if(fabs(temp_vehicle_s - estimate.s) > sensor_fusion_range) {
      double a = temp_vehicle.s + map->max_s - estimate.s;
      double b = estimate.s - (temp_vehicle.s - map->max_s);

      if(a < sensor_fusion_range)
        temp_vehicle_s = temp_vehicle.s + map->max_s;
      else
        temp_vehicle_s = temp_vehicle.s - map->max_s;
    }

    if (temp_vehicle.lane == lane.lane_id && temp_vehicle_s > estimate.s && temp_vehicle_s < min_s) {
      min_s = temp_vehicle_s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

vector<double> Planning::Jmt(vector< double> start, vector <double> end, double t)
{
  Eigen::Matrix3f A;
  Eigen::Vector3f b;
  A << t*t*t,t*t*t*t,t*t*t*t*t,  3*t*t,4*t*t*t,5*t*t*t*t,  6*t,12*t*t,20*t*t*t;
  b << end[0]-(start[0]+start[1]*t+0.5*start[2]*t*t), end[1]-(start[1]+start[2]*t), end[2]-start[2];
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

  return {start[0],start[1],0.5*start[2],x[0],x[1],x[2]};
}

vector<double> Planning::Jmt_t(vector<double> coeffs, double t)
{
  float s = coeffs[0] + coeffs[1]*t + coeffs[2]*t*t + coeffs[3]*t*t*t + coeffs[4]*t*t*t*t + coeffs[5]*t*t*t*t*t;
  float s_dot = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t*t + 4*coeffs[4]*t*t*t + 5*coeffs[5]*t*t*t*t;
  float s_dot2 = 2*coeffs[2] + 6*coeffs[3]*t + 12*coeffs[4]*t*t + 20*coeffs[5]*t*t*t;

  return {s, s_dot, s_dot2};
}

bool Planning::IsOnTrack(float d, Lane const& lane){
  return fabs(d - lane.lane_center_d < 1);
}

float Planning::CalculateCost(Trajectory const& trajectory, vector<PredictionEstimate> const& predictions)
{
  float cost = 0.0;

  cost += planning_cost_acceleration_weight * AccelerationLimitCost(trajectory, vehicle_max_acceleration);
  cost += planning_cost_speed_limit_weight * SpeedLimitCost(trajectory, vehicle_speed_limit);
  cost += planning_cost_off_lane_weight * OffTrackCost(trajectory);
  cost += planning_cost_distance_to_obstacle * DistanceToObstacle(trajectory, predictions);
  cost += planning_cost_inefficiency_cost * InefficiencyCost(trajectory, vehicle_speed_limit);

  return cost;
}

float Planning::AccelerationLimitCost(Trajectory const& trajectory, float acceleration_limit)
{
  for(int i=0; i<trajectory.path_point_list.size(); ++i){
    if(trajectory.path_point_list[i].s_dot2 > acceleration_limit)
      return 1;
  }
  return 0;
}

float Planning::SpeedLimitCost(Trajectory const& trajectory, float speed_limit)
{
  for(int i=0; i<trajectory.path_point_list.size(); ++i){
    if(trajectory.path_point_list[i].s_dot > speed_limit)
      return 1;
  }
  return 0;
}

float Planning::OffTrackCost(Trajectory const& trajectory)
{
  float final_d = trajectory.path_point_list.back().d;
  Lane lane = map->GetLaneByLateralCoord(final_d);

  return 1/(1+exp(-(final_d-lane.lane_center_d)*(final_d-lane.lane_center_d)));
}

float Planning::DistanceToObstacle(Trajectory const& trajectory, vector<PredictionEstimate> predictions)
{
  float min = 9999999;
  float x_ego, y_ego, x_other, y_other, dist;

  for(int i=0; i<trajectory.path_point_list.size(); ++i){
    x_ego = trajectory.path_point_list[i].x;
    y_ego = trajectory.path_point_list[i].y;

    for(int j=0; j<predictions.size(); ++j){
      if(predictions[j].trajectory.path_point_list.size() > i){
        x_other = predictions[j].trajectory.path_point_list[i].x;
        y_other = predictions[j].trajectory.path_point_list[i].y;

        dist = (x_other-x_ego)*(x_other-x_ego) + (y_other-y_ego)*(y_other-y_ego);
        if(min > dist){
          min = dist;
        }
      }
    }
  }

  //if distance smaller than safe follow distance
  return exp(-max(dist-planning_collison_radius, 0.f));
}

float Planning::InefficiencyCost(Trajectory const& trajectory, float target_speed)
{
  float final_speed = trajectory.path_point_list.back().s_dot;
  return 1/(1+exp(-(final_speed-target_speed) * (final_speed-target_speed)));
}
