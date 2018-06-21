//
// Created by tom on 6/11/18.
//

#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include "classifier.h"
#include "../vehicle.h"
#include "../map/map.h"
#include "../sensor_fusion/sensor_fusion.h"

using namespace std;

class PredictionEstimate{
public:
  int id;
  float probabilty;
  Trajectory trajectory;
};


class Prediction {
public:
  Prediction(Map const* map);
  ~Prediction();
  void OnSensorFusionUpdate(vector<SensorFusionMessage> const& vehicle_list);
  vector<PredictionEstimate> GetPrediction();
private:
  Map const* map;
  void initClassifier();
  GNB state_classifier;
  vector<SensorFusionMessage> vehicle_list_snapshot;

  bool PredictVehiclesTrajectory(vector<SensorFusionMessage> const& vehicle_list,
                                 vector<PredictionEstimate>& trajectory_list);
};


#endif //PATH_PLANNING_PREDICTION_H
