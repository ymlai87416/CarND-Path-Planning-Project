//
// Created by tom on 6/13/18.
//

#include "planning.h"
#include "../Constants.h"
#include <sstream>
#include "../../Eigen-3.3/Eigen/Dense"
#include "../../spline.h"
#include "../../utility.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Planning::Planning(Map const* map)
{
  this-> map = map;
  this->state = "KL";
  this->target_lane = 1;
  this->process_frame = 0;
}

Planning::~Planning()
{

}

void Planning::OnLocalizationUpdate(LocalizationEstimate const& estimate)
{
  this->prev_estimate_snapshot = this->local_estimate_snapshot;
  this->local_estimate_snapshot = estimate;
}

void Planning::OnPredictionUpdate(vector<SensorFusionMessage> const& vehicle_list,
                                  vector<PredictionEstimate> const& vehicle_trajectories)
{
  this->vehicle_list_snapshot = vehicle_list;
  this->predict_trajectory_snapshot = vehicle_trajectories;
}

PlanningResult Planning::PlanTrajectory()
{
  this->process_frame += 1;

  PlanningResult result;

  //LocalizationEstimate estimate = this->local_estimate_snapshot;
  LocalizationEstimate estimate = this->local_estimate_snapshot;
  vector<SensorFusionMessage> vehicle_list = this->vehicle_list_snapshot;
  vector<PredictionEstimate> predictions = this->predict_trajectory_snapshot;

  if(!IsCurrentTrackSafe(estimate, predictions)){
    //if current track is not safe, replan the track by shorten the future track to 5.
    estimate.previous_path_x.resize(5);
    estimate.previous_path_y.resize(5);
    target_lane = map->GetLaneByLateralCoord(estimate.d).lane_id;
    cout << "Track is not safe, replan the track!!" << endl;
  }

  vector<string> states = GetSuccessorStates(target_lane);
  vector<float> costs;
  vector<string> final_states;
  vector<Trajectory> final_trajectories;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    TrajectoryCost traj_cost;
    traj_cost.state = *it;

    Trajectory trajectory = GenerateTrajectory(*it, estimate, vehicle_list);
    if (trajectory.path_point_list.size() != 0) {
      traj_cost.total_score = CalculateCost(trajectory, vehicle_list, predictions, traj_cost.sub_score);
      costs.push_back(traj_cost.total_score);
      final_trajectories.push_back(trajectory);
    }

    result.cost.push_back(traj_cost);
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  //convert to next state
  state = final_trajectories[best_idx].state;
  target_lane = final_trajectories[best_idx].target_lane;

  //cout << "Best trajectory found: state: " << state << " lane: " << target_lane << endl;

  result.best_trajectory = final_trajectories[best_idx];

  return result;
}

vector<float> Planning::GetKinematics(VehicleState const& estimate,
                                      vector<SensorFusionMessage> const& vehicle_list, int lane)
{
  float max_velocity_accel_limit = (vehicle_max_acceleration + estimate.a) * planning_time_length + estimate.speed;
  float min_velocity_accel_limit = max(0.01f, (vehicle_max_deceleration + estimate.a) * planning_time_length + estimate.speed);
  float new_position;
  float new_velocity;
  float new_accel;
  SensorFusionMessage vehicle_ahead;
  SensorFusionMessage vehicle_behind;

  if (GetVehicleAhead(estimate.s, vehicle_list, lane, vehicle_ahead)) {
    float vehicle_ahead_s =vehicle_ahead.s + estimate.relative_time * vehicle_ahead.s_dot;
    if(estimate.s - vehicle_ahead_s > sensor_fusion_range)
      vehicle_ahead_s += map->max_s;

    if (GetVehicleBehind(estimate.s, vehicle_list, lane, vehicle_behind)) {
      float vehicle_behind_s = vehicle_behind.s + estimate.relative_time * vehicle_behind.s_dot;

      if(estimate.s < vehicle_behind_s) //case when the behind car pass the max_s line.
        vehicle_behind_s -= map->max_s;

      //defensive driving, keep 2 second distance in-front and behind, if behind does not make it, compensate in-front.
      float safety_distance = planning_safe_follow_distance_in_sec * 2 * estimate.speed;
      safety_distance -= max(0.f, min(planning_safe_follow_distance_in_sec * estimate.speed, estimate.s - vehicle_behind_s));


      float max_velocity_in_front = ((vehicle_ahead_s - estimate.s - safety_distance)
                                     - 0.5 * estimate.a * planning_time_length * planning_time_length) / planning_time_length
                                    + vehicle_ahead.s_dot ;

      new_velocity = max(min(min(max_velocity_in_front, max_velocity_accel_limit), vehicle_speed_limit), min_velocity_accel_limit);

      /*
      cout << "front car @" << vehicle_ahead_s << " this @" << estimate.s << " behind car @" << vehicle_behind_s << endl;
      cout << "car detected in both front and behind s:" << safety_distance
           << " v: " << max_velocity_in_front << " new v: " << new_velocity << endl;
      */
      //new_velocity = vehicle_ahead.s_dot; //must travel at the speed of traffic, regardless of preferred buffer
      //cout << "Cars detected in both front and behind v_front: " << vehicle_ahead.s_dot
      //     << " v_behind: " << vehicle_behind.s_dot << " v: " << new_velocity << endl;
    } else {
      //currently acceleartion is assume to be 0.
      float safety_distance = planning_safe_follow_distance_in_sec * estimate.speed;

      float max_velocity_in_front = ((vehicle_ahead_s - estimate.s - safety_distance)
                                    - 0.5 * estimate.a * planning_time_length * planning_time_length) / planning_time_length
                                    + vehicle_ahead.s_dot ;
      new_velocity = max(min(min(max_velocity_in_front, max_velocity_accel_limit), vehicle_speed_limit), min_velocity_accel_limit);

      //cout << "Cars detected in front v_front: " << vehicle_ahead.s_dot << " v: " << new_velocity << endl;
    }
  } else {
    new_velocity = min(max_velocity_accel_limit, vehicle_speed_limit);
    //cout << "Empty lane: " << new_velocity << endl;
  }

  new_accel = (new_velocity - estimate.speed) / planning_time_length; //Equation: (v_1 - v_0)/t = acceleration
  new_position = estimate.s + new_velocity * planning_time_length + new_accel / 2.0 * planning_time_length * planning_time_length;

  return{new_position, new_velocity, new_accel};
}

bool Planning::GetVehicleBehind(float s,vector<SensorFusionMessage> const& vehicle_list,
                                int target_lane, SensorFusionMessage & rVehicle){
  int max_s = -1;
  bool found_vehicle = false;
  Lane lane= map->GetLane(target_lane);

  SensorFusionMessage temp_vehicle;
  for (vector<SensorFusionMessage>::const_iterator it = vehicle_list.begin(); it != vehicle_list.end(); ++it) {
    temp_vehicle = *it;

    float temp_vehicle_s = temp_vehicle.s;
    if(fabs(temp_vehicle_s - s) > sensor_fusion_range) {
      double a = temp_vehicle.s + map->max_s - s;
      double b = s - (temp_vehicle.s - map->max_s);

      if(a < sensor_fusion_range)
        temp_vehicle_s = temp_vehicle.s + map->max_s;
      else
        temp_vehicle_s = temp_vehicle.s - map->max_s;
    }

    if (temp_vehicle.lane == lane.lane_id && temp_vehicle_s < s && temp_vehicle_s > max_s) {
      max_s = temp_vehicle_s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

bool Planning::GetVehicleAhead(float s, vector<SensorFusionMessage> const& vehicle_list,
                               int target_lane, SensorFusionMessage & rVehicle){

  int min_s = 99999999;
  bool found_vehicle = false;
  Lane lane=  map->GetLane(target_lane);

  SensorFusionMessage temp_vehicle;
  for (vector<SensorFusionMessage>::const_iterator it = vehicle_list.begin(); it != vehicle_list.end(); ++it) {
    temp_vehicle = *it;

    float temp_vehicle_s = temp_vehicle.s;
    if(fabs(temp_vehicle_s - s) > sensor_fusion_range) {
      double a = temp_vehicle.s + map->max_s - s;
      double b = s - (temp_vehicle.s - map->max_s);

      if(a < sensor_fusion_range)
        temp_vehicle_s = temp_vehicle.s + map->max_s;
      else
        temp_vehicle_s = temp_vehicle.s - map->max_s;
    }

    if (temp_vehicle.lane == lane.lane_id && temp_vehicle_s > s && temp_vehicle_s < min_s) {
      min_s = temp_vehicle_s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

bool Planning::FeasibilityCheck(Trajectory trajectory)
{
  bool result = true;
  for(int i=0; i<trajectory.path_point_list.size(); ++i){
    /*
    if(trajectory.path_point_list[i].s_dot > 50){
      result = false;
      cerr << "Frame: " << this->process_frame << " point " << i << " breach s_dot limit!! value: " << trajectory.path_point_list[i].s_dot << endl;
    }*/
    if(trajectory.path_point_list[i].v > 50){
      result = false;
      cerr << "Frame: " << this->process_frame << " point " << i << " breach speed limit!! value: " << trajectory.path_point_list[i].v << endl;
    }
    if(trajectory.path_point_list[i].a > 10){
      result = false;
      cerr << "Frame: " << this->process_frame << " point " << i << " breach max acceleration!! value: " << trajectory.path_point_list[i].a << endl;
    }
    if(trajectory.path_point_list[i].a < -10){
      result = false;
      cerr << "Frame: " << this->process_frame << " point " << i << " breach max barking!! value: " << trajectory.path_point_list[i].a << endl;
    }
  }
  return result;
}

Trajectory Planning::GenerateTrajectory(string state, LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list)
{
  Trajectory trajectory;
  if (state.compare("KL") == 0) {
    trajectory = KeepLaneTrajectory(estimate, vehicle_list);
  } else if (state.compare("LCL") == 0){
    trajectory = LaneChangeTrajectory(estimate, vehicle_list, state, this->target_lane-1);
  } else if (state.compare("LCR") == 0) {
    trajectory = LaneChangeTrajectory(estimate, vehicle_list, state, this->target_lane+1);
  } else if (state.compare("PLCL") == 0){
    trajectory = PrepLaneChangeTrajectory(estimate, vehicle_list, state, this->target_lane-1);
  } else if (state.compare("PLCR") == 0) {
    trajectory = PrepLaneChangeTrajectory(estimate, vehicle_list, state, this->target_lane+1);
  }
  return trajectory;
}

void Planning::FindActualVelocityAndAcceleration(vector<PathPoint>& path_point_list)
{

  for(int i=0, j=1; j<path_point_list.size(); ++i, ++j){
    float dist = sqrt((path_point_list[j].x - path_point_list[i].x) * (path_point_list[j].x - path_point_list[i].x) +
        (path_point_list[j].y - path_point_list[i].y) * (path_point_list[j].y - path_point_list[i].y));
    path_point_list[i].v = dist / update_interval;
  }
  path_point_list.back().v = 0;

  for(int i=0, j=1; j<path_point_list.size()-1; ++i, ++j){
    path_point_list[i].a = (path_point_list[j].v - path_point_list[i].v) / update_interval;
  }

  vector<PathPoint>::reverse_iterator it = path_point_list.rbegin();
  it->a = 0;
  ++it;
  it->a = 0;
}

void __planning__init_state(LocalizationEstimate const& estimate, const Map* const map,
                            VehicleState& state, vector<double>& ptsx, vector<double>& ptsy){

  if(estimate.previous_path_x.size() > 4) {
    float relative_time = (estimate.previous_path_x.size() - 1) * trajectory_discretize_timestep;
    auto x_it = estimate.previous_path_x.rbegin();
    auto y_it = estimate.previous_path_y.rbegin();

    double point_x[3], point_y[3];
    double speed[2];
    double a;

    for(int i=2; i>=0; --i){
      point_x[i] = *x_it;
      point_y[i] = *y_it;
      ++x_it; ++y_it;
    }

    for(int i=0; i<2; ++i){
      float dist = sqrt((point_x[i+1]-point_x[i])*(point_x[i+1]-point_x[i])
                        + (point_y[i+1]-point_y[i])*(point_y[i+1]-point_y[i]) );
      speed[i] = dist / trajectory_discretize_timestep;
    }

    a = (speed[1] - speed[0]) / trajectory_discretize_timestep;

    float yaw = atan2(point_y[1]-point_y[0], point_x[1]-point_x[0]);
    vector<double> frenet = map->GetFrenet(point_x[0], point_y[0], yaw);

    a = min((double)vehicle_max_acceleration, max(a, (double)vehicle_max_deceleration));

    state = VehicleState(relative_time, point_x[0], point_y[0], yaw, speed[0], frenet[0], frenet[1], a);

    x_it++;
    y_it++;
    ptsx.push_back(*x_it);
    ptsx.push_back(point_x[0]);

    ptsy.push_back(*y_it);
    ptsy.push_back(point_y[0]);

    /*
    cout << "debug: " << point_x[0] << " " << point_y[0] << " " << speed[0] << " " << a << endl;
    if(fabs(a) > 10)
      cout << "Acceleration exceed the limit...." << endl;
    */
  }
  else{
    double prev_car_x = estimate.x - cos(estimate.yaw);
    double prev_car_y = estimate.y - sin(estimate.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(estimate.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(estimate.y);

    state = VehicleState(0, estimate.x, estimate.y, estimate.yaw, estimate.speed, estimate.s, estimate.d, 0);
  }
}

vector<PathPoint> __planning__generate_path_point(LocalizationEstimate estimate,
                                                  VehicleState const& state, vector<float> kinematics,
                                                  Lane const& target_lane,
                                                  vector<double>& ptsx, vector<double>& ptsy,
                                                  const Map * const map){
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];

  vector<double> next_wp0 = map->GetXY(state.s + 0.5*(state.speed + new_v) * 1.3, target_lane.lane_center_d);
  vector<double> next_wp1 = map->GetXY(state.s + 0.5*(state.speed + new_v) * 1.5, target_lane.lane_center_d);
  vector<double> next_wp2 = map->GetXY(state.s + 0.5*(state.speed + new_v) * 2, target_lane.lane_center_d);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for(int i=0; i<ptsx.size(); ++i){

    double x_dash, y_dash;

    ConvertXYToVehicleCoordination(ptsx[i], ptsy[i], state.x, state.y, state.yaw, x_dash, y_dash);

    ptsx[i] = x_dash;
    ptsy[i] = y_dash;
  }

  tk::spline s;

  s.set_points(ptsx, ptsy);

  vector<PathPoint> path_point_list;
  float rt = 0;

  if(estimate.previous_path_x.size() > 0) {
    for (int i = 0; i < estimate.previous_path_x.size() - 2; ++i) {
      rt += trajectory_discretize_timestep;
      PathPoint path_point;
      path_point.relative_time = rt;
      path_point.x = estimate.previous_path_x[i];
      path_point.y = estimate.previous_path_y[i];

      path_point_list.push_back(path_point);
    }

    /*cout << "debug: connect pt " << estimate.previous_path_x[estimate.previous_path_x.size() - 2]
         << estimate.previous_path_y[estimate.previous_path_y.size() - 2] << endl; */
  }

  size_t pp = 49-path_point_list.size();
  //cout << "debug: add " << pp << " point to trajectory" << endl;


  double target_x = 0.5 * (state.speed + new_v) * pp * trajectory_discretize_timestep;
  double target_y = s(target_x);
  double target_distance = sqrt(target_x*target_x + target_y*target_y);

  float rt_s = 0;
  for(int i=1; i<=pp; ++i){
    rt += trajectory_discretize_timestep; rt_s += trajectory_discretize_timestep;
    PathPoint path_point;

    double tan_dist = state.speed * rt_s + 0.5 * new_a * rt_s * rt_s;
    double x_dash = tan_dist / target_distance * target_x;
    double y_dash = s(x_dash);

    double x_point, y_point;

    ConvertVehicleCoordinateToXY(x_dash, y_dash, state.x, state.y, state.yaw, x_point, y_point);

    /*
    if(i==1)
      cout << "debug: connect pt " << x_point <<  y_point<< endl;
      */

    path_point.relative_time = rt;
    path_point.x = x_point;
    path_point.y = y_point;

    path_point_list.push_back(path_point);
  }

  return path_point_list;
}

float __planning__calculate_path_length(vector<PathPoint> const& path_point_list){
  float path_length =0 ;
  float prev_x, prev_y, x, y, dx, dy;
  x = path_point_list[0].x; y=path_point_list[0].y;

  for(vector<PathPoint>::const_iterator it=path_point_list.begin(); it!= path_point_list.end(); ++it){
    prev_x = x;
    prev_y = y;
    x = it->x;
    y = it->y;
    dx = x - prev_x; dy = y - prev_y;
    path_length += sqrt(dx*dx+dy*dy);
  }
  return path_length;
}

Trajectory Planning::KeepLaneTrajectory(LocalizationEstimate const& estimate,
                                        vector<SensorFusionMessage> const& vehicle_list)
{
  Trajectory trajectory;
  std::ostringstream stringStream;
  Lane current_lane = map->GetLane(this->target_lane);
  stringStream << "Keep Lane" << current_lane.lane_id;
  trajectory.debug_state = stringStream.str();
  trajectory.state = "KL";
  trajectory.target_lane = current_lane.lane_id;
  trajectory.x = estimate.x; trajectory.y = estimate.y; trajectory.yaw = estimate.yaw;

  VehicleState vehicle_state;
  vector<double> ptsx, ptsy;

  __planning__init_state(estimate, map, vehicle_state, ptsx, ptsy);

  vector<float> kinematics = GetKinematics(vehicle_state, vehicle_list, current_lane.lane_id);
  trajectory.target_s = kinematics[0]; trajectory.target_v = kinematics[1]; trajectory.target_a = kinematics[2];

  vector<PathPoint> path_point_list = __planning__generate_path_point(estimate, vehicle_state, kinematics, current_lane, ptsx, ptsy, map);

  FindActualVelocityAndAcceleration(path_point_list);
  trajectory.path_point_list = path_point_list;

  trajectory.total_path_time = trajectory.path_point_list.size() * trajectory_discretize_timestep;
  trajectory.total_path_length = __planning__calculate_path_length(trajectory.path_point_list);

  FeasibilityCheck(trajectory);

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
  trajectory.x = estimate.x; trajectory.y = estimate.y; trajectory.yaw = estimate.yaw;

  VehicleState vehicle_state;
  vector<double> ptsx, ptsy;

  __planning__init_state(estimate, map, vehicle_state, ptsx, ptsy);

  SensorFusionMessage next_lane_vehicle;
  for (vector<SensorFusionMessage>::const_iterator it = vehicle_list.begin(); it != vehicle_list.end(); ++it) {
    next_lane_vehicle = *it;
    if (fabs(next_lane_vehicle.s - estimate.s) < planning_safe_follow_distance_in_sec*estimate.speed && next_lane_vehicle.lane == target_lane_id) {
      //If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }

  vector<float> kinematics = GetKinematics(vehicle_state, vehicle_list, current_lane.lane_id);
  trajectory.target_s = kinematics[0]; trajectory.target_v = kinematics[1]; trajectory.target_a = kinematics[2];

  vector<PathPoint> path_point_list = __planning__generate_path_point(estimate, vehicle_state, kinematics, target_lane, ptsx, ptsy, map);

  FindActualVelocityAndAcceleration(path_point_list);
  trajectory.path_point_list = path_point_list;

  trajectory.total_path_time = trajectory.path_point_list.size() * trajectory_discretize_timestep;
  trajectory.total_path_length = __planning__calculate_path_length(trajectory.path_point_list);

  FeasibilityCheck(trajectory);

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
  trajectory.target_lane = current_lane.lane_id;
  trajectory.x = estimate.x; trajectory.y = estimate.y; trajectory.yaw = estimate.yaw;

  VehicleState vehicle_state;
  vector<double> ptsx, ptsy;
  SensorFusionMessage vehicle_behind;
  float new_s;
  float new_v;
  float new_a;

  __planning__init_state(estimate, map, vehicle_state, ptsx, ptsy);

  vector<float> curr_lane_new_kinematics =
      GetKinematics(vehicle_state, vehicle_list, current_lane.lane_id);

  if (GetVehicleBehind(vehicle_state.s, vehicle_list, current_lane.lane_id, vehicle_behind)) {
    //Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];

  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = GetKinematics(vehicle_state, vehicle_list, target_lane_id);
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

  trajectory.target_s = new_s; trajectory.target_v = new_v; trajectory.target_a = new_a;

  vector<PathPoint> path_point_list = __planning__generate_path_point(estimate, vehicle_state, {new_s, new_v, new_a}, current_lane, ptsx, ptsy, map);

  FindActualVelocityAndAcceleration(path_point_list);
  trajectory.path_point_list = path_point_list;

  trajectory.total_path_time = trajectory.path_point_list.size() * trajectory_discretize_timestep;
  trajectory.total_path_length = __planning__calculate_path_length(trajectory.path_point_list);

  FeasibilityCheck(trajectory);

  return trajectory;
}

vector<string> Planning::GetSuccessorStates(int lane_id){
  Lane lane = map->GetLane(lane_id);

  vector<string> states;

  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("KL");
    if (lane.lane_id != 0)
      states.push_back("PLCL");
    if (lane.lane_id != map->lanes_available - 1)
      states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    states.push_back("KL");
    if (lane.lane_id != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    states.push_back("KL");
    if (lane.lane_id != map->lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0){
      states.push_back("KL");
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

bool Planning::IsCurrentTrackSafe(LocalizationEstimate const& estimate,
                                  vector<PredictionEstimate> const& predictions){
  float min = 9999999;
  float x_ego, y_ego, x_other, y_other, dist;

  for(int i=0; i<estimate.previous_path_x.size(); ++i){
    x_ego = estimate.previous_path_x[i];
    y_ego = estimate.previous_path_x[i];
    float relative_time = trajectory_discretize_timestep * (i+1);

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
  return min > planning_collison_radius;
}

float Planning::CalculateCost(Trajectory const& trajectory,
                              vector<SensorFusionMessage> const& vehicle_list,
                              vector<PredictionEstimate> const& predictions,
                              vector<float>& cost_breakdown)
{
  float cost = 0.0;

  cost_breakdown.push_back(planning_cost_acceleration_weight * AccelerationLimitCost(trajectory, vehicle_max_acceleration));
  cost_breakdown.push_back(planning_cost_speed_limit_weight * SpeedLimitCost(trajectory, vehicle_speed_limit));
  cost_breakdown.push_back(planning_cost_off_lane_weight * OffTrackCost(trajectory));
  cost_breakdown.push_back(planning_cost_distance_to_obstacle * DistanceToObstacle(trajectory, predictions));
  cost_breakdown.push_back(planning_cost_inefficiency_cost * InefficiencyCost(trajectory, vehicle_list, vehicle_speed_limit));
  cost_breakdown.push_back(planning_cost_keep_lane_bonus * KeepLaneBonus(trajectory));
  cost_breakdown.push_back(planning_cost_change_free_lane_bonus * ChangeToFreeLaneBouns(trajectory, vehicle_list));

  for(auto it=cost_breakdown.begin(); it != cost_breakdown.end(); ++it)
    cost += *it;

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
  float min_dist = 9999999;
  float x_ego, y_ego, x_other, y_other, dist;

  int base = 0;
  float rt = trajectory.path_point_list[0].relative_time;
  for(int i=0; i<predictions[0].trajectory.path_point_list.size(); ++i){
    float rt_pred = predictions[0].trajectory.path_point_list[0].relative_time;

    if(fabs(rt_pred - rt) < 1e-6 || rt_pred > rt){
      base = i;
      break;
    }
  }

  for(int i=0; i<trajectory.path_point_list.size(); ++i){
    x_ego = trajectory.path_point_list[i].x;
    y_ego = trajectory.path_point_list[i].y;

    for(int j=0; j<predictions.size(); ++j){
      if(predictions[j].trajectory.path_point_list.size() > i){
        x_other = predictions[j].trajectory.path_point_list[i+base].x;
        y_other = predictions[j].trajectory.path_point_list[i+base].y;

        dist = (x_other-x_ego)*(x_other-x_ego) + (y_other-y_ego)*(y_other-y_ego);
        if(min_dist > dist){
          min_dist = dist;
        }
      }
    }
  }

  //if distance smaller than safe follow distance
  return exp(-max(min_dist-planning_collison_radius, 0.f));
}

float Planning::InefficiencyCost(Trajectory const& trajectory, vector<SensorFusionMessage> const& predictions,
                                 float target_speed)
{
  /*
  float final_speed = trajectory.path_point_list.back().s_dot;
  return 1/(1+exp(-(final_speed-target_speed) * (final_speed-target_speed)));
   */
  /*
  Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
  You can use the lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function to determine the speed
  for a lane. This function is very similar to what you have already implemented in the "Implement a Second Cost Function in C++" quiz.
  */

  int intended_lane;
  if (trajectory.state.compare("PLCL") == 0) {
    intended_lane = trajectory.target_lane - 1;
  } else if (trajectory.state.compare("PLCR") == 0) {
    intended_lane = trajectory.target_lane + 1;
  } else {
    intended_lane = trajectory.target_lane;
  }

  vector<double> frenet = map->GetFrenet(trajectory.x, trajectory.y, trajectory.yaw);
  SensorFusionMessage rVehicle;

  bool result = GetVehicleAhead(frenet[0], predictions, intended_lane, rVehicle);

  float proposed_speed_intended, proposed_speed_final;

  //If no vehicle is in the proposed lane, we can travel at target speed.
  if (result) {
    /*
    float rVehicle_s = rVehicle.s;
    if(rVehicle.s - frenet[0] > sensor_fusion_range){
      rVehicle_s += map->max_s;
    }

    if(rVehicle_s - frenet[0] < planning_inefficiency_max_range)
      proposed_speed_intended = rVehicle.s_dot;
    else
      proposed_speed_intended = target_speed;
      */
    proposed_speed_intended = rVehicle.s_dot;
  }
  else
    proposed_speed_intended = target_speed;

  result = GetVehicleAhead(frenet[0], predictions, trajectory.target_lane, rVehicle);
  if (result)
    proposed_speed_final = rVehicle.s_dot;
  else
    proposed_speed_final = target_speed;

  float cost = (2.0*target_speed - proposed_speed_intended - proposed_speed_final)/target_speed;

  return cost;
}

float Planning::KeepLaneBonus(Trajectory const &trajectory) {
  if(trajectory.state.compare("KL") == 0)
    return 0;
  else
    return 1;
}

float Planning::ChangeToFreeLaneBouns(Trajectory const &trajectory, vector<SensorFusionMessage> const &predictions){
  if (trajectory.state.compare("PLCL") == 0 || trajectory.state.compare("PLCR") == 0) {

    int intended_lane;

    if (trajectory.state.compare("PLCL") == 0) {
      intended_lane = trajectory.target_lane - 1;
    } else if (trajectory.state.compare("PLCR") == 0) {
      intended_lane = trajectory.target_lane + 1;
    }

    vector<double> frenet = map->GetFrenet(trajectory.x, trajectory.y, trajectory.yaw);
    SensorFusionMessage rVehicle;

    bool result = GetVehicleAhead(frenet[0], predictions, intended_lane, rVehicle);

    float cost=0;

    //If no vehicle is in the proposed lane, we can travel at target speed.
    if (result) {
      float rVehicle_s = rVehicle.s;
      if (rVehicle.s - frenet[0] > sensor_fusion_range) {
        rVehicle_s += map->max_s;
      }

      if(rVehicle_s - frenet[0] < planning_inefficiency_max_range)
        cost= 1;
      else
        cost= 0;
    }
    else
      cost= 0;

    return cost;
  }
  else
    return 1;
}