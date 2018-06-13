//
// Created by tom on 6/11/18.
//
#ifndef UTILITY_H
#define UTILITY_H

double pi();
double deg2rad(double x);
double rad2deg(double x);

/*
vector<Vehicle> create_vehicles_from_sensor_fusion(vector<vector<double>> input){
  vector<Vehicle> result;
  int id;
  float x, y, yaw, speed, s, d;
  for(int i=0; i<input.size(); ++i){
    id = input[0];
    x = input[1]; y = input[2];
    yaw = input[3]; speed = input[4];
    s = input[5]; d = input[6];
    result.push_back(new Vehicle(s, d, speed, 0, x, y, yaw));
  }
  return result;
}

Vehicle create_ego_from_localization(double x, double y, double yaw, double speed, double s, double d){
  return new Vehicle(s, d, speed, 0, x, y, yaw);
}
*/

#endif