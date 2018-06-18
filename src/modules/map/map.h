//
// Created by tom on 6/11/18.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H
#include <string>
#include <vector>

using namespace std;

class Lane
{
public:
  int lane_id;
  float lane_left_d;
  float lane_right_d;
  float lane_center_d;
};

class Map {
public:
  Map(string map_file_, float max_s, int lanes_available);
  ~Map();
  void ReadMap(string map_file_, float max_s, int lanes_available);
  vector<double> GetXY(double s, double d) const;
  vector<double> GetXY(double s, double d, double s_dot, double d_dot) const;
  vector<double> GetFrenet(double x, double y, double theta) const;
  vector<double> GetFrenet(double x, double y, double theta, double speed) const;
  Lane GetLane(int lane_id) const;
  Lane GetLaneByLateralCoord(float d) const;

  float max_s;
  int lanes_available;

private:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  double Distance(double x1, double y1, double x2, double y2) const;
  int ClosestWaypoint(double x, double y) const;
  int NextWaypoint(double x, double y, double theta) const;
};


#endif //PATH_PLANNING_MAP_H

