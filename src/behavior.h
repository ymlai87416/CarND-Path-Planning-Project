//
// Created by tom on 6/11/18.
//

#ifndef PATH_PLANNING_BEHAVIOR_H
#define PATH_PLANNING_BEHAVIOR_H

#include "vehicle.h"

class Behavior {
public:
  Behavior();
  ~Behavior();

  string ChooseNextState(map<int, vector<Vehicle>> trajectories);
private:
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
};


#endif //PATH_PLANNING_BEHAVIOR_H
