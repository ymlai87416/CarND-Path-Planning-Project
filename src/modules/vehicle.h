#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "Constants.h"

using namespace std;

class PathPoint
{
public:
  float x;
  float y;
  float relative_time;
  float theta;
  float s;
  float d;

  //aux data
  float s_dot;
  float d_dot;
  float s_dot2;
  float d_dot2;

  float v;
  float a;

  PathPoint(){};

  /*
  PathPoint(float x, float y, float theta, float v, float a, float s, float d, float relative_time){
    this->x = x; this->y =y; this->v = v; this->a = a; this->theta = theta;
    this->s = s; this->d = d; this->relative_time = relative_time;
  }
   */
};

class Trajectory{
public:
  float total_path_length;
  float total_path_time;
  string state;
  int target_lane;
  string debug_state;

  float x;
  float y;
  float yaw;

  float target_s;
  float target_v;
  float target_a;

  vector<PathPoint> path_point_list;
};


#endif