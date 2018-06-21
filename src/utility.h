//
// Created by tom on 6/11/18.
//
#ifndef UTILITY_H
#define UTILITY_H


double pi();
double deg2rad(double x);
double rad2deg(double x);
double mph2mps(double mph);
double mps2mph(double mps);
void ConvertXYToVehicleCoordination(double x, double y, const double x_ref, const double y_ref, const double theta, double& x_dash, double& y_dash);
void ConvertVehicleCoordinateToXY(double x_dash, double y_dash, const double x_ref, const double y_ref, const double theta, double& x, double& y);
long int unix_timestamp();
#endif