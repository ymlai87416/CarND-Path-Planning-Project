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


class Planning
{
public:
  Planning(Map const* map);
  ~Planning();
  void UpdateLocalization(LocalizationEstimate estimate);
  void UpdatePrediction(vector<SensorFusionMessage> vehicle, vector<PredictionEstimate> vehicle_trajectories);
  Trajectory PlanTrajectory();

  Trajectory KeepLaneTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list);
  Trajectory LaneChangeTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list, string state, int target_lane_id);
  Trajectory PrepLaneChangeTrajectory(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list, string state, int target_lane_id);

private:
  vector<PredictionEstimate> predict_trajectory_snapshot;
  vector<SensorFusionMessage> vehicle_list_snapshot;
  LocalizationEstimate local_estimate_snapshot;
  Map const * map;
  string state;
  int target_lane;

  //trajectory generation
  bool FeasibilityCheck(Trajectory trajectory);
  vector<float> GetKinematics(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list, int lane);
  Trajectory GenerateTrajectory(string state, LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& vehicle_list);
  bool CheckForCollision(Trajectory trajectory);
  vector<string> GetSuccessorStates(LocalizationEstimate const& estimate);
  bool GetVehicleBehind(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& predictions, int target_lane, SensorFusionMessage & rVehicle);
  bool GetVehicleAhead(LocalizationEstimate const& estimate, vector<SensorFusionMessage> const& predictions, int target_lane, SensorFusionMessage & rVehicle);
  vector<double> Jmt(vector<double> start, vector<double> end, double t);
  vector<double> Jmt_t(vector<double> coeffs, double t);
  void FindActualVelocityAndAcceleration(vector<PathPoint>& path_point_list);
  bool IsOnTrack(float d, Lane const& lane);

  //cost functions
  float CalculateCost(Trajectory const& trajectory, vector<PredictionEstimate> const& predictions);
  float AccelerationLimitCost(Trajectory const& trajectory, float acceleration_limit);
  float SpeedLimitCost(Trajectory const& trajectory, float speed_limit);
  float OffTrackCost(Trajectory const& trajectory);
  float DistanceToObstacle(Trajectory const& trajectory, vector<PredictionEstimate> predictions);
  float InefficiencyCost(Trajectory const& trajectory, float target_speed);
};


#endif //PATH_PLANNING_PLANNING_H
