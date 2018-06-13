//
// Created by tom on 6/11/18.
//

#include "map.h"
#include "utility.h"
#include <fstream>
#include <sstream>
#include <cmath>

using namespace std;

Map::Map(string map_file_)
{
  readMap(map_file_);
}

Map::~Map()
{
}

void Map::readMap(string map_file_)
{
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    this->map_waypoints_x.push_back(x);
    this->map_waypoints_y.push_back(y);
    this->map_waypoints_s.push_back(s);
    this->map_waypoints_dx.push_back(d_x);
    this->map_waypoints_dy.push_back(d_y);
  }
}

vector<double> Map::getXY(double s, double d)
{
  int prev_wp = -1;

  while(s > this->map_waypoints_s[prev_wp+1] && (prev_wp < (int)(this->map_waypoints_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%this->map_waypoints_x.size();

  double heading = atan2((this->map_waypoints_y[wp2]-this->map_waypoints_y[prev_wp]),(this->map_waypoints_x[wp2]-this->map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-this->map_waypoints_s[prev_wp]);

  double seg_x = this->map_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = this->map_waypoints_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> Map::getFrenet(double x, double y, double theta)
{
  int next_wp = NextWaypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = this->map_waypoints_x.size()-1;
  }

  double n_x = this->map_waypoints_x[next_wp]-this->map_waypoints_x[prev_wp];
  double n_y = this->map_waypoints_y[next_wp]-this->map_waypoints_y[prev_wp];
  double x_x = x - this->map_waypoints_x[prev_wp];
  double x_y = y - this->map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-this->map_waypoints_x[prev_wp];
  double center_y = 2000-this->map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(this->map_waypoints_x[i],this->map_waypoints_y[i],this->map_waypoints_x[i+1],this->map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

vector<double> Map::getFrenet(double x, double y, double theta, double speed)
{
  int next_wp = NextWaypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = this->map_waypoints_x.size()-1;
  }

  double n_x = this->map_waypoints_x[next_wp]-this->map_waypoints_x[prev_wp];
  double n_y = this->map_waypoints_y[next_wp]-this->map_waypoints_y[prev_wp];
  double x_x = x - this->map_waypoints_x[prev_wp];
  double x_y = y - this->map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  //find the s_dot and d_dot
  double v_x = speed * cos(theta);
  double v_y = speed * sin(theta);
  double proj_s_dot_norm = (v_x*n_x+v_y*n_y)/(n_x*n_x+n_y*n_y);
  double s_dot_x = proj_s_dot_norm * n_x;
  double s_dot_y = proj_s_dot_norm * n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);
  double s_dot =distance(0, 0, s_dot_x, s_dot_y);
  double d_dot =distance(v_x, v_y, s_dot_x, s_dot_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-this->map_waypoints_x[prev_wp];
  double center_y = 2000-this->map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  //see if d_dot value is positive or negative by comparing it to a center point

  center_x = 1000-this->map_waypoints_x[prev_wp];
  center_y = 2000-this->map_waypoints_y[prev_wp];
  centerToPos = distance(center_x,center_y,v_x,v_y);
  centerToRef = distance(center_x,center_y,s_dot_x,s_dot_y);

  if(centerToPos <= centerToRef)
  {
    d_dot *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(this->map_waypoints_x[i],this->map_waypoints_y[i],this->map_waypoints_x[i+1],this->map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d, s_dot, d_dot};
}

double Map::distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Map::ClosestWaypoint(double x, double y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < this->map_waypoints_x.size(); i++)
  {
    double map_x = this->map_waypoints_x[i];
    double map_y = this->map_waypoints_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta)
{
  int closestWaypoint = ClosestWaypoint(x,y);

  double map_x = this->map_waypoints_x[closestWaypoint];
  double map_y = this->map_waypoints_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == this->map_waypoints_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}