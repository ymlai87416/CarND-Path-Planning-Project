//
// Created by tom on 6/11/18.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H
#include <string>
#include <vector>

using namespace std;

class Map {
public:
  Map(string map_file_);
  ~Map();
  void readMap(string map_file_);
  vector<double> getXY(double s, double d);
  vector<double> getFrenet(double x, double y, double theta);
  vector<double> getFrenet(double x, double y, double theta, double speed);

private:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  double distance(double x1, double y1, double x2, double y2);
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
};


#endif //PATH_PLANNING_MAP_H
