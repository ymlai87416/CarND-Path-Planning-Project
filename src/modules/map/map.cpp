//
// Created by tom on 6/11/18.
//

#include "map.h"
#include "../../utility.h"
#include <fstream>
#include <sstream>
#include <cmath>

using namespace std;

Map::Map(string map_file_, float max_s, int lanes_available)
{
  ReadMap(map_file_, max_s, lanes_available);
}

Map::~Map()
{
}

void Map::ReadMap(string map_file_, float max_s, int lanes_available)
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

  this->max_s = max_s;
  this->lanes_available = lanes_available;
}

vector<double> Map::GetXY(double s, double d) const
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

vector<double> Map::GetXY(double s, double d, double s_dot, double d_dot) const
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

  double vx = s_dot * cos(heading) + d_dot * cos(perp_heading);
  double vy = s_dot * sin(heading) + d_dot * sin(perp_heading);
  double theta = atan2(vy, vx);

  return {x,y,theta};
}


vector<double> Map::GetFrenet(double x, double y, double theta) const
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

  double frenet_d = Distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-this->map_waypoints_x[prev_wp];
  double center_y = 2000-this->map_waypoints_y[prev_wp];
  double centerToPos = Distance(center_x,center_y,x_x,x_y);
  double centerToRef = Distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += Distance(this->map_waypoints_x[i],this->map_waypoints_y[i],this->map_waypoints_x[i+1],this->map_waypoints_y[i+1]);
  }

  frenet_s += Distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

vector<double> Map::GetFrenet(double x, double y, double theta, double speed) const
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

  double frenet_d = Distance(x_x,x_y,proj_x,proj_y);
  double s_dot =Distance(0, 0, s_dot_x, s_dot_y);
  double d_dot =Distance(v_x, v_y, s_dot_x, s_dot_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-this->map_waypoints_x[prev_wp];
  double center_y = 2000-this->map_waypoints_y[prev_wp];
  double centerToPos = Distance(center_x,center_y,x_x,x_y);
  double centerToRef = Distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  //see if d_dot value is positive or negative by comparing it to a center point

  center_x = 1000-this->map_waypoints_x[prev_wp];
  center_y = 2000-this->map_waypoints_y[prev_wp];
  centerToPos = Distance(center_x,center_y,v_x,v_y);
  centerToRef = Distance(center_x,center_y,s_dot_x,s_dot_y);

  if(centerToPos <= centerToRef)
  {
    d_dot *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += Distance(this->map_waypoints_x[i],this->map_waypoints_y[i],this->map_waypoints_x[i+1],this->map_waypoints_y[i+1]);
  }

  frenet_s += Distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d, s_dot, d_dot};
}

double Map::Distance(double x1, double y1, double x2, double y2) const
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Map::ClosestWaypoint(double x, double y) const
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < this->map_waypoints_x.size(); i++)
  {
    double map_x = this->map_waypoints_x[i];
    double map_y = this->map_waypoints_y[i];
    double dist = Distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta) const
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

Lane Map::GetLane(int lane_id) const{
  Lane result;
  result.lane_id = lane_id;
  result.lane_center_d =2+4*lane_id;
  result.lane_left_d = 2+4*lane_id-2;
  result.lane_right_d = 2+4*lane_id+2;

  return result;
}

Lane Map::GetLaneByLateralCoord(float d) const{
  Lane result;
  result.lane_id = d / 4;
  result.lane_center_d =2+4*result.lane_id;
  result.lane_left_d = 2+4*result.lane_id-2;
  result.lane_right_d = 2+4*result.lane_id+2;

  return result;
}

