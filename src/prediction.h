//
// Created by tom on 6/11/18.
//

#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include "classifier.h"
#include "vehicle.h"
#include <map>

using namespace std;

class Prediction {
public:
  Prediction();
  ~Prediction();
  map<int, vector<Vehicle>> predict_vehicles_trajectory(vector<Vehicle> vehicle_list);
private:
  void init_classifier();
  GNB state_classifier;
};


#endif //PATH_PLANNING_PREDICTION_H
