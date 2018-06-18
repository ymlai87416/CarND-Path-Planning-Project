//
// Created by tom on 6/11/18.
//
#include "utility.h"
#include <cmath>

double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2mps(double mph) { return mph * 0.44704; }
double mps2mph(double mps) { return mps * 2.23694; }

void ConvertXYToVehicleCoordination(double x, double y, const double x_ref, const double y_ref, const double theta, double& x_dash, double& y_dash){
  double shift_x = x - x_ref;
  double shift_y = y - y_ref;

  x_dash = (shift_x*cos(0-theta)-shift_y*sin(0-theta));
  y_dash = (shift_x*sin(0-theta)+shift_y*cos(0-theta));
}

void ConvertVehicleCoordinateToXY(double x_dash, double y_dash, const double x_ref, const double y_ref, const double theta, double& x, double& y){
  x = (x_dash * cos(theta) - y_dash*sin(theta));
  y = (x_dash * sin(theta) + y_dash*cos(theta));

  x += x_ref;
  y += y_ref;
}