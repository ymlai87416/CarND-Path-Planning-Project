//
// Created by tom on 6/17/18.
//

#ifndef PATH_PLANNING_LOCALIZATION_H
#define PATH_PLANNING_LOCALIZATION_H

#include <vector>
using namespace std;

class LocalizationEstimate{
public:
  float x;
  float y;
  float s;
  float d;
  float yaw;
  float speed;

  //calculated properties
  float ax;
  float ay;
  float s_dot;
  float d_dot;
  float s_dot2;
  float d_dot2;

  vector<float> previous_path_x;
  vector<float> previous_path_y;

  double end_path_s;
  double end_path_d;
};

#endif //PATH_PLANNING_LOCALIZATION_H
