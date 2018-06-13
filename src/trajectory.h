//
// Created by tom on 6/11/18.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include "vehicle.h"
#include <string>

using namespace std;

//Accept input from localization, prediction and behavior to output a trajectory
//responsible for execute the command safetly. avoid collision
class Trajectory {
public:
  Trajectory();
  ~Trajectory();

  void generate_trajectory(string state);
private:
  void jmt();
  void feasibility_check();
};


#endif //PATH_PLANNING_TRAJECTORY_H
