//
// Created by tom on 6/13/18.
//

#ifndef PATH_PLANNING_PLANNING_H
#define PATH_PLANNING_PLANNING_H

#include "../vehicle.h"
#include "../localization/localization.h"
#include "../sensor_fusion/sensor_fusion.h"
#include "../prediction/prediction.h"
#include <vector>
#include <string>
#include "../map/map.h"

using namespace std;

class VehicleState{
public:
  float relative_time;
  float s;
  float d;

  float x;
  float y;
  float yaw;
  float speed;

  float a;

  VehicleState() {};

  VehicleState(float rt, float x, float y, float yaw, float speed, float s, float d, float a):
      relative_time(rt), x(x), y(y), yaw(yaw), speed(speed), s(s), d(d), a(a){};
};

class TrajectoryCost{
public:
  string state;
  vector<float> sub_score;
  float total_score;
};

class PlanningResult{
public:
  Trajectory best_trajectory;
  vector<TrajectoryCost> cost;
};

class Planning
{
public:
  Planning(Map const* map);
  ~Planning();
  void OnLocalizationUpdate(LocalizationEstimate const& estimate);
  void OnPredictionUpdate(vector<SensorFusionMessage> const& vehicle, vector<PredictionEstimate> const& vehicle_trajectories);
  PlanningResult PlanTrajectory();

  Trajectory KeepLaneTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list);
  Trajectory LaneChangeTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list, string state, int target_lane_id);
  Trajectory PrepLaneChangeTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list, string state, int target_lane_id);

private:
  vector<PredictionEstimate> predict_trajectory_snapshot;
  vector<SensorFusionMessage> vehicle_list_snapshot;
  LocalizationEstimate local_estimate_snapshot;
  LocalizationEstimate prev_estimate_snapshot;
  Map const * map;
  string state;
  int target_lane;
  int process_frame;

  //trajectory generation
  bool FeasibilityCheck(Trajectory trajectory);
  vector<float> GetKinematics(VehicleState const& estimate, vector<SensorFusionMessage> const& vehicle_list, int lane);
  bool GetVehicleBehind(float s, vector<SensorFusionMessage> const& predictions, int target_lane, SensorFusionMessage & rVehicle);
  bool GetVehicleAhead(float s, vector<SensorFusionMessage> const& predictions, int target_lane, SensorFusionMessage & rVehicle);
  Trajectory GenerateTrajectory(string state, LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list);
  vector<string> GetSuccessorStates(int lane_id);
  void FindActualVelocityAndAcceleration(vector<PathPoint>& path_point_list);
  bool IsCurrentTrackSafe(LocalizationEstimate const& estimate, vector<PredictionEstimate> const& predictions);

  //cost functions
  float CalculateCost(Trajectory const& trajectory, vector<SensorFusionMessage> const& vehicle_list,
                      vector<PredictionEstimate> const& predictions, vector<float>& cost_breakdown);
  float AccelerationLimitCost(Trajectory const& trajectory, float acceleration_limit);
  float SpeedLimitCost(Trajectory const& trajectory, float speed_limit);
  float OffTrackCost(Trajectory const& trajectory);
  float DistanceToObstacle(Trajectory const& trajectory, vector<PredictionEstimate> predictions);
  float InefficiencyCost(Trajectory const& trajectory, vector<SensorFusionMessage> const& predictions, float target_speed);
  float KeepLaneBonus(Trajectory const& trajectory);
  float ChangeToFreeLaneBouns(Trajectory const &trajectory, vector<SensorFusionMessage> const &predictions);
};


#endif //PATH_PLANNING_PLANNING_H
