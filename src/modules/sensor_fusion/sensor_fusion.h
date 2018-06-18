//
// Created by tom on 6/17/18.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H

class SensorFusionMessage{

public:
  int id;
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float s;
  float d;
  float s_dot;
  float d_dot;
  float lane;
};

#endif //PATH_PLANNING_SENSOR_FUSION_H
