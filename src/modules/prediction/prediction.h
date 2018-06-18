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
  bool PredictVehiclesTrajectory(vector<SensorFusionMessage> vehicle_list,
        vector<PredictionEstimate>& trajectory_list);
private:
  Map const* map;
  void initClassifier();
  GNB state_classifier;
};


#endif //PATH_PLANNING_PREDICTION_H
